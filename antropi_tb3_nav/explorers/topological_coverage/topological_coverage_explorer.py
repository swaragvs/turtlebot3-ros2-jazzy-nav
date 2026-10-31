#!/usr/bin/env python3
"""
Topological coverage explorer: one node that explores, covers and recovers.

Pipeline (one decision per goal, no competing controllers):

  1. Build a planning grid from /map: free & clear of walls (inflated by the
     robot footprint), optionally clipped to a bounding box.
  2. Wavefront-flood that grid from the robot -> only REACHABLE cells exist.
  3. Targets, in priority order:
       a) frontier clusters (reachable free cells touching unknown space)
       b) coverage gaps (reachable free cells the LiDAR has never seen)
     scored by path distance, turn cost and cluster size.
  4. Send ONE Nav2 goal.  While driving, the goal is dropped early if it was
     explored on the way (no wasted travel).
  5. Pause exploration when TF localization becomes stale or jumps, and resume
     only after the pose is fresh and stable.
  6. Anti-stuck ladder:
       stall/timeout/reject -> blacklist goal
       3 failures in a row  -> reset counter, hold, and retry after Nav2 recovery
       nothing left         -> clear blacklist and retry (limited rounds)
       still nothing        -> save graph and finish.

/cmd_vel is used only for zero-velocity safety stops; Nav2 owns motion.
"""
import json
import math
import os
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

KEY_OFF = 50000
KEY_SPAN = 100000


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def dilate(mask, radius):
    """Binary dilation with a disk of `radius` cells (pure NumPy)."""
    if radius <= 0:
        return mask.copy()
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if (dy == 0 and dx == 0) or dy * dy + dx * dx > radius * radius:
                continue
            y0, y1 = max(0, dy), min(h, h + dy)
            x0, x1 = max(0, dx), min(w, w + dx)
            if y0 >= y1 or x0 >= x1:
                continue
            out[y0:y1, x0:x1] |= mask[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    return out


def box3(mask):
    """8-neighbour growth of a boolean mask."""
    a = mask.copy()
    a[1:, :] |= mask[:-1, :]
    a[:-1, :] |= mask[1:, :]
    b = a.copy()
    b[:, 1:] |= a[:, :-1]
    b[:, :-1] |= a[:, 1:]
    return b


class TopologicalCoverageExplorer(Node):
    PLAN = 0
    NAVIGATE = 1
    RECOVER = 2
    FINISHED = 3
    LOCALIZATION_HOLD = 4

    def __init__(self):
        super().__init__('topological_coverage_explorer')

        # ---------------- parameters ----------------
        d = self.declare_parameter
        d('base_frame', 'base_footprint')
        d('linear_speed', 0.15)
        d('publish_speed_limit', True)
        d('robot_radius', 0.22)
        d('safety_margin', 0.08)
        d('minimum_goal_distance', 0.80)
        d('near_goal_distance', 0.35)
        d('min_frontier_cells', 3)
        d('turn_weight', 0.30)            # metres of cost per radian of turn
        d('size_weight', 0.40)            # metres of bonus per metre of frontier
        d('goal_blacklist_radius', 0.50)
        d('visited_goal_radius', 0.35)
        d('node_merge_radius', 0.50)
        d('sensed_range', 3.0)
        d('coverage_cell_size', 0.10)
        d('coverage_min_cells', 6)
        d('visit_mark_radius', 0.50)
        d('planning_period', 0.30)
        d('goal_timeout_base', 20.0)
        d('goal_stall_timeout', 8.0)
        d('stall_distance', 0.10)
        d('goal_check_period', 2.0)
        d('min_commit_time', 4.0)         # never preempt a goal younger than this
        d('preempt_min_distance', 1.2)    # only preempt if still this far away
        d('goal_cooldown', 1.0)           # pause after a failed/cancelled goal
        d('ack_timeout', 6.0)             # Nav2 must accept a goal within this
        d('no_progress_timeout', 20.0)    # no net approach to goal -> fail
        d('resize_settle_time', 3.0)      # wait after the /map grid grows
        d('comfort_radius', 0.30)         # prefer goals this far from walls
        d('scan_timeout', 1.0)
        d('pose_timeout', 1.0)
        d('pose_jump_distance', 0.75)
        d('pose_jump_angle', 1.20)
        d('pose_stability_distance', 0.20)
        d('pose_stability_angle', 0.35)
        d('pose_stability_duration', 1.0)
        d('max_relax_level', 1)
        d('max_retry_rounds', 2)
        d('no_target_retry_period', 5.0)
        d('recovery_pause_time', 2.0)
        d('use_bounds', False)
        d('x_min', -7.2)
        d('x_max', 7.2)
        d('y_min', -7.2)
        d('y_max', 7.2)
        d('resume_graph', False)
        d('save_period', 10.0)
        d('graph_save_path',
          os.path.expanduser('~/antropi_ws/coverage_graph.json'))

        self.radius = self.pf('robot_radius')
        self.margin = self.pf('safety_margin')
        self.cov_size = max(0.05, self.pf('coverage_cell_size'))

        # ---------------- ROS interfaces ----------------
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/exploration_graph', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- state ----------------
        self.latest_scan = None
        self.latest_map = None
        self.map_version = 0
        self.grid_cache = None
        self.last_scan_time = None
        self.last_seen_update = 0.0
        self.pose = None
        self.yaw = 0.0
        self.last_pose_time = None
        self.last_tf_stamp_ns = None
        self.last_tf_stamp_advance = None
        self.localization_hold = False
        self.localization_candidate = None
        self.localization_candidate_yaw = 0.0
        self.localization_stability_anchor = None
        self.localization_stability_anchor_yaw = 0.0
        self.localization_stable_since = None
        self.pose_unavailable_since = None

        self.state = self.PLAN
        self.goal = None
        self.goal_handle = None
        self.gen = 0
        self.active_gen = None
        self.goal_counter = 0
        self.progress_pose = None
        self.progress_yaw = 0.0
        self.progress_time = 0.0
        self.last_goal_check = 0.0
        self.last_plan_time = 0.0
        self.last_speed_pub = 0.0
        self.map_sig = None
        self.map_resized_at = -1e9
        self.cooldown_until = 0.0
        self.nav_down_since = None
        self.nav_down_reported = False
        self.ack_failures = 0
        self.best_goal_dist = 0.0
        self.best_goal_time = 0.0

        self.relax_level = 0
        self.retry_rounds = 0
        self.consecutive_failures = 0
        self.blacklist = []
        self.visited_goals = []
        self.seen = set()
        self._seen_arr = np.zeros(0, dtype=np.int64)
        self._seen_arr_len = -1

        self.nodes = []
        self.edges = []
        self.last_node_id = None

        self.rec_t0 = 0.0

        self.dirty = False
        if self.pb('resume_graph'):
            self.load_graph()

        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_markers)
        self.create_timer(self.pf('save_period'), self.periodic_save)
        self.get_logger().info('Topological coverage explorer initialized')

    # ---------------- parameter helpers ----------------
    def pf(self, name):
        return float(self.get_parameter(name).value)

    def pb(self, name):
        return bool(self.get_parameter(name).value)

    def clock_seconds(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    # ---------------- subscriptions ----------------
    def scan_cb(self, msg):
        self.latest_scan = msg
        self.last_scan_time = self.clock_seconds()
        now = self.last_scan_time
        if now - self.last_seen_update >= 0.4:   # ~2.5 Hz is plenty
            self.last_seen_update = now
            self.update_seen()

    def map_cb(self, msg):
        self.latest_map = msg
        self.map_version += 1
        i = msg.info
        sig = (i.width, i.height, round(i.origin.position.x, 3),
               round(i.origin.position.y, 3))
        if self.map_sig is not None and sig != self.map_sig:
            # SLAM grew the map -> Nav2's global costmap is being resized now.
            # Goals sent/running during this moment tend to fail or hang Nav2.
            self.map_resized_at = self.clock_seconds()
        self.map_sig = sig

    # ---------------- pose ----------------
    def update_pose(self):
        frames = [str(self.get_parameter('base_frame').value), 'base_link']
        transform = None
        for frame in frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', frame, rclpy.time.Time())
            except TransformException:
                continue
            break
        if transform is None:
            self.get_logger().warn('Waiting for map->base TF',
                                   throttle_duration_sec=5.0)
            return False

        now = self.clock_seconds()
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns > 0:
            if (self.last_tf_stamp_ns is None or
                    stamp_ns != self.last_tf_stamp_ns):
                self.last_tf_stamp_ns = stamp_ns
                self.last_tf_stamp_advance = now
            elif (self.last_tf_stamp_advance is not None and
                  now - self.last_tf_stamp_advance > self.pf('pose_timeout')):
                self.get_logger().warn(
                    'map->base TF is stale; pausing exploration',
                    throttle_duration_sec=5.0)
                self.enter_localization_hold('stale map->base TF', now=now)
                return False

        q = transform.transform.rotation
        measured_pose = (transform.transform.translation.x,
                         transform.transform.translation.y)
        measured_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not all(math.isfinite(value) for value in
                   (*measured_pose, measured_yaw)):
            self.enter_localization_hold('non-finite TF pose', now=now)
            return False
        self.pose_unavailable_since = None

        if self.localization_hold:
            self.update_localization_candidate(
                measured_pose, measured_yaw, now)
            return True

        if self.pose is not None:
            distance = math.hypot(measured_pose[0] - self.pose[0],
                                  measured_pose[1] - self.pose[1])
            angle = abs(wrap(measured_yaw - self.yaw))
            if (distance > self.pf('pose_jump_distance') or
                    angle > self.pf('pose_jump_angle')):
                self.enter_localization_hold(
                    'implausible TF pose jump', measured_pose,
                    measured_yaw, now)
                return True

        self.pose = measured_pose
        self.yaw = measured_yaw
        self.last_pose_time = now
        return True

    def update_localization_candidate(self, pose, yaw, now):
        if self.localization_candidate is None:
            self.localization_candidate = pose
            self.localization_candidate_yaw = yaw
            self.localization_stability_anchor = pose
            self.localization_stability_anchor_yaw = yaw
            self.localization_stable_since = now
            self.last_pose_time = now
            return

        distance = math.hypot(
            pose[0] - self.localization_stability_anchor[0],
            pose[1] - self.localization_stability_anchor[1])
        angle = abs(wrap(yaw - self.localization_stability_anchor_yaw))
        if (distance > self.pf('pose_stability_distance') or
                angle > self.pf('pose_stability_angle')):
            self.localization_stable_since = now
            self.localization_stability_anchor = pose
            self.localization_stability_anchor_yaw = yaw
        self.localization_candidate = pose
        self.localization_candidate_yaw = yaw
        self.last_pose_time = now

    def enter_localization_hold(self, reason, candidate=None, yaw=0.0,
                                now=None):
        if self.state == self.FINISHED:
            return
        now = self.clock_seconds() if now is None else now
        if not self.localization_hold:
            self.localization_hold = True
            self.goal = None
            self.cancel_active()
            self.stop()
            self.state = self.LOCALIZATION_HOLD
            self.get_logger().warn(
                f'Localization unreliable ({reason}); cancelling goal and '
                'holding until TF stabilizes')
        self.localization_candidate = candidate
        self.localization_candidate_yaw = yaw
        self.localization_stability_anchor = candidate
        self.localization_stability_anchor_yaw = yaw
        self.localization_stable_since = now if candidate is not None else None
    def localization_hold_step(self, now):
        self.stop()
        if (self.localization_candidate is None or
                self.localization_stable_since is None or
                self.last_pose_time is None or
                now - self.last_pose_time > self.pf('pose_timeout')):
            return
        if (now - self.localization_stable_since <
                self.pf('pose_stability_duration')):
            return
        self.pose = self.localization_candidate
        self.yaw = self.localization_candidate_yaw
        self.localization_hold = False
        self.localization_candidate = None
        self.localization_stability_anchor = None
        self.localization_stable_since = None
        self.pose_unavailable_since = None
        self.state = self.PLAN
        self.get_logger().info(
            'TF localization is fresh and stable; resuming exploration')

    # ---------------- coverage ("seen by LiDAR") ----------------
    def cov_keys(self, x, y):
        i = np.floor(np.asarray(x) / self.cov_size).astype(np.int64) + KEY_OFF
        j = np.floor(np.asarray(y) / self.cov_size).astype(np.int64) + KEY_OFF
        return i * KEY_SPAN + j

    def seen_array(self):
        if self._seen_arr_len != len(self.seen):
            self._seen_arr = np.fromiter(self.seen, dtype=np.int64,
                                         count=len(self.seen))
            self._seen_arr_len = len(self.seen)
        return self._seen_arr

    def mark_disk(self, x, y, radius):
        n = int(math.ceil(radius / self.cov_size))
        offs = np.arange(-n, n + 1) * self.cov_size
        gx, gy = np.meshgrid(offs, offs)
        m = np.hypot(gx, gy) <= radius
        keys = self.cov_keys(x + gx[m], y + gy[m])
        self.seen.update(np.unique(keys).tolist())
        self.dirty = True

    def update_seen(self):
        scan = self.latest_scan
        if (scan is None or self.pose is None or self.localization_hold or
                self.last_pose_time is None or
                self.clock_seconds() - self.last_pose_time >
                self.pf('pose_timeout')):
            return
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return
        stride = max(1, n // 360)
        r = ranges[::stride]
        ang = scan.angle_min + np.arange(0, n, stride) * scan.angle_increment
        hit = np.isfinite(r)
        r = np.where(np.isposinf(r), scan.range_max, r)
        ok = np.isfinite(r) & (r > 0.05)
        r, ang, hit = r[ok], ang[ok], hit[ok]
        if r.size == 0:
            return
        sensed = self.pf('sensed_range')
        reach = np.minimum(np.where(hit, r - 0.10, r), sensed)
        t = np.arange(0.15, sensed, self.cov_size * 0.8)
        mask = t[None, :] <= reach[:, None]
        th = self.yaw + ang
        xs = self.pose[0] + np.cos(th)[:, None] * t[None, :]
        ys = self.pose[1] + np.sin(th)[:, None] * t[None, :]
        keys = np.unique(self.cov_keys(xs[mask], ys[mask]))
        self.seen.update(keys.tolist())
        self.mark_disk(self.pose[0], self.pose[1], 0.30)

    # ---------------- planning grid ----------------
    def build_grids(self, level):
        key = (self.map_version, level)
        if self.grid_cache is not None and self.grid_cache['key'] == key:
            return self.grid_cache
        info = self.latest_map.info
        h, w = info.height, info.width
        res = info.resolution
        data = np.asarray(self.latest_map.data, dtype=np.int16).reshape(h, w)
        free = (data >= 0) & (data < 25)
        occ = data >= 65
        unknown = data < 0

        inflation = max(self.radius + 0.005,
                        self.radius + self.margin - 0.025 * level)
        blocked = dilate(occ, int(math.ceil(inflation / res)))
        trav = free & ~blocked

        ox, oy = info.origin.position.x, info.origin.position.y
        if self.pb('use_bounds'):
            xs = ox + (np.arange(w) + 0.5) * res
            ys = oy + (np.arange(h) + 0.5) * res
            bx = (xs > self.pf('x_min')) & (xs < self.pf('x_max'))
            by = (ys > self.pf('y_min')) & (ys < self.pf('y_max'))
            trav &= by[:, None] & bx[None, :]

        comfort = trav & ~dilate(
            occ, int(math.ceil(self.pf('comfort_radius') / res)))
        unk_adj = np.zeros_like(unknown)
        unk_adj[1:, :] |= unknown[:-1, :]
        unk_adj[:-1, :] |= unknown[1:, :]
        unk_adj[:, 1:] |= unknown[:, :-1]
        unk_adj[:, :-1] |= unknown[:, 1:]

        self.grid_cache = {
            'key': key, 'h': h, 'w': w, 'res': res, 'ox': ox, 'oy': oy,
            'data': data, 'trav': trav, 'unk_adj': unk_adj,
            'comfort': comfort,
        }
        return self.grid_cache

    def world_to_cell(self, g, x, y):
        col = int(math.floor((x - g['ox']) / g['res']))
        row = int(math.floor((y - g['oy']) / g['res']))
        if 0 <= row < g['h'] and 0 <= col < g['w']:
            return row, col
        return None

    def cell_to_world(self, g, row, col):
        return (g['ox'] + (col + 0.5) * g['res'],
                g['oy'] + (row + 0.5) * g['res'])

    def start_cell(self, g):
        cell = self.world_to_cell(g, self.pose[0], self.pose[1])
        if cell is None:
            return None
        trav = g['trav']
        if trav[cell]:
            return cell
        r, c = cell
        m = 10
        r0, r1 = max(0, r - m), min(g['h'], r + m + 1)
        c0, c1 = max(0, c - m), min(g['w'], c + m + 1)
        idx = np.argwhere(trav[r0:r1, c0:c1])
        if idx.size == 0:
            trav[cell] = True          # robot is wedged: force-start here
            return cell
        idx = idx + np.array([r0, c0])
        k = int(np.argmin(((idx - np.array([r, c])) ** 2).sum(axis=1)))
        return int(idx[k][0]), int(idx[k][1])

    @staticmethod
    def flood(trav, start):
        """8-connected wavefront distance (in cells); -1 = unreachable."""
        h, w = trav.shape
        dist = np.full((h, w), -1, dtype=np.int32)
        rows = np.flatnonzero(trav.any(axis=1))
        cols = np.flatnonzero(trav.any(axis=0))
        if rows.size == 0:
            return dist
        r0, r1 = rows[0], rows[-1] + 1
        c0, c1 = cols[0], cols[-1] + 1
        sub = trav[r0:r1, c0:c1]
        sd = np.full(sub.shape, -1, dtype=np.int32)
        front = np.zeros_like(sub)
        sr, sc = start[0] - r0, start[1] - c0
        if not (0 <= sr < sub.shape[0] and 0 <= sc < sub.shape[1]):
            return dist
        front[sr, sc] = True
        reached = front.copy()
        sd[sr, sc] = 0
        for step in range(1, sub.shape[0] + sub.shape[1] + 2):
            grown = box3(front) & sub & ~reached
            if not grown.any():
                break
            sd[grown] = step
            reached |= grown
            front = grown
        dist[r0:r1, c0:c1] = sd
        return dist

    # ---------------- target selection ----------------
    def excluded(self, x, y):
        bl = self.pf('goal_blacklist_radius')
        vr = self.pf('visited_goal_radius')
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < bl:
                return True
        for vx, vy in self.visited_goals:
            if math.hypot(x - vx, y - vy) < vr:
                return True
        return False

    def score(self, path_m, x, y, size_m, penalty=0.0):
        heading = math.atan2(y - self.pose[1], x - self.pose[0])
        turn = abs(wrap(heading - self.yaw))
        return (path_m + penalty + self.pf('turn_weight') * turn
                - self.pf('size_weight') * min(size_m, 2.0))

    def frontier_targets(self, g, dist, min_dist):
        res = g['res']
        rows, cols = np.nonzero((dist >= 0) & g['unk_adj'])
        if rows.size == 0:
            return []
        cells = set(zip(rows.tolist(), cols.tolist()))
        min_cells = int(self.pf('min_frontier_cells'))
        targets = []
        while cells:
            seed = cells.pop()
            comp, stack = [seed], [seed]
            while stack:
                r, c = stack.pop()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nb = (r + dr, c + dc)
                        if nb in cells:
                            cells.remove(nb)
                            comp.append(nb)
                            stack.append(nb)
            if len(comp) < min_cells:
                continue
            arr = np.asarray(comp)
            centroid = arr.mean(axis=0)
            comf = g['comfort'][arr[:, 0], arr[:, 1]]
            pool = arr[comf] if comf.any() else arr
            pool_x = g['ox'] + (pool[:, 1] + 0.5) * res
            pool_y = g['oy'] + (pool[:, 0] + 0.5) * res
            far_enough = (
                np.hypot(pool_x - self.pose[0], pool_y - self.pose[1]) >=
                self.pf('near_goal_distance'))
            pool = pool[far_enough]
            if pool.size == 0:
                continue
            k = int(np.argmin(((pool - centroid) ** 2).sum(axis=1)))
            r, c = int(pool[k][0]), int(pool[k][1])
            path_m = float(dist[r, c]) * res * 1.15
            if path_m < min_dist:
                continue
            x, y = self.cell_to_world(g, r, c)
            if self.excluded(x, y):
                continue
            size_m = len(comp) * res
            pen = 0.0 if g['comfort'][r, c] else 0.4
            targets.append({'x': x, 'y': y, 'kind': 'frontier',
                            'path': path_m,
                            'score': self.score(path_m, x, y, size_m, pen)})
        targets.sort(key=lambda t: t['score'])
        return targets

    def coverage_targets(self, g, dist, min_dist):
        res = g['res']
        rows, cols = np.nonzero(dist >= 0)
        if rows.size == 0:
            return []
        xs = g['ox'] + (cols + 0.5) * res
        ys = g['oy'] + (rows + 0.5) * res
        keys = self.cov_keys(xs, ys)
        seen = self.seen_array()
        unseen = (~np.isin(keys, seen)) if seen.size else np.ones(
            keys.shape, dtype=bool)
        if not unseen.any():
            return []
        rows, cols, xs, ys = rows[unseen], cols[unseen], xs[unseen], ys[unseen]
        bins = (np.floor(xs).astype(np.int64) * 100003
                + np.floor(ys).astype(np.int64))
        _, inv = np.unique(bins, return_inverse=True)
        order = np.argsort(inv, kind='stable')
        counts = np.bincount(inv)
        groups = np.split(order, np.cumsum(counts)[:-1])
        min_cells = int(self.pf('coverage_min_cells'))
        targets = []
        for idx in groups:
            if idx.size < min_cells:
                continue
            cx, cy = xs[idx].mean(), ys[idx].mean()
            comf = g['comfort'][rows[idx], cols[idx]]
            pool = idx[comf] if comf.any() else idx
            far_enough = np.hypot(
                xs[pool] - self.pose[0],
                ys[pool] - self.pose[1]) >= self.pf('near_goal_distance')
            pool = pool[far_enough]
            if pool.size == 0:
                continue
            k = int(np.argmin((xs[pool] - cx) ** 2 + (ys[pool] - cy) ** 2))
            j = pool[k]
            r, c = int(rows[j]), int(cols[j])
            path_m = float(dist[r, c]) * res * 1.15
            if path_m < min_dist:
                continue
            x, y = float(xs[j]), float(ys[j])
            if self.excluded(x, y):
                continue
            size_m = math.sqrt(idx.size) * self.cov_size
            pen = 0.0 if g['comfort'][r, c] else 0.4
            targets.append({'x': x, 'y': y, 'kind': 'coverage',
                            'path': path_m,
                            'score': self.score(path_m, x, y, size_m, pen)})
        targets.sort(key=lambda t: t['score'])
        return targets

    def select_target(self, g, dist):
        near = self.pf('near_goal_distance')
        far = self.pf('minimum_goal_distance')
        for finder, min_d in ((self.frontier_targets, far),
                              (self.frontier_targets, near),
                              (self.coverage_targets, far),
                              (self.coverage_targets, near)):
            found = finder(g, dist, min_d)
            if found:
                return found[0]
        return None

    # ---------------- planning & dispatch ----------------
    def plan_and_dispatch(self):
        if self.latest_map is None or self.pose is None:
            return
        now_p = self.clock_seconds()
        if now_p < self.cooldown_until:          # let Nav2 settle
            return
        if now_p - self.map_resized_at < self.pf('resize_settle_time'):
            return                               # costmap is resizing
        if not self.nav_client.server_is_ready():
            return
        g = self.build_grids(self.relax_level)
        start = self.start_cell(g)
        if start is None:
            self.get_logger().warn('Robot outside map; running recovery')
            self.start_recovery()
            return
        dist = self.flood(g['trav'], start)
        target = self.select_target(g, dist)

        if target is None:
            if self.relax_level < int(self.pf('max_relax_level')):
                self.relax_level += 1
                self.get_logger().info(
                    f'No targets; relaxing footprint (level {self.relax_level})')
                return
            if self.retry_rounds < int(self.pf('max_retry_rounds')):
                self.retry_rounds += 1
                self.relax_level = 0
                self.blacklist.clear()
                self.get_logger().info(
                    f'No targets; clearing blacklist (round {self.retry_rounds})')
                return
            self.relax_level = 0
            self.retry_rounds = 0
            self.blacklist.clear()
            self.cooldown_until = (
                now_p + self.pf('no_target_retry_period'))
            self.get_logger().warn(
                'No reachable frontier or coverage targets after retries; '
                'waiting for map updates before trying again',
                throttle_duration_sec=15.0)
            return

        heading = (math.atan2(target['y'] - self.pose[1],
                              target['x'] - self.pose[0])
                   if target['path'] > 0.1 else self.yaw)
        if not self.send_goal(target['x'], target['y'], heading):
            return
        now = self.clock_seconds()
        timeout = (self.pf('goal_timeout_base') +
                   target['path'] / max(0.05, 0.5 * self.pf('linear_speed')))
        self.goal = dict(target, t0=now, timeout=timeout, accepted=False)
        self.best_goal_dist = math.hypot(target['x'] - self.pose[0],
                                         target['y'] - self.pose[1])
        self.best_goal_time = now
        self.progress_pose = self.pose
        self.progress_yaw = self.yaw
        self.progress_time = now
        self.last_goal_check = now
        self.state = self.NAVIGATE
        self.goal_counter += 1
        self.get_logger().info(
            f'Goal #{self.goal_counter} [{target["kind"]}] '
            f'({target["x"]:.2f}, {target["y"]:.2f}) path~{target["path"]:.1f} m')

    def send_goal(self, x, y, heading):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        # A zero stamp asks TF consumers to use the latest transform, avoiding
        # clock-domain mismatches when this node and Nav2 use different clocks.
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)
        self.gen += 1
        gen = self.gen
        self.active_gen = gen
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, g=gen: self.on_goal_response(f, g))
        return True

    def on_goal_response(self, future, gen):
        try:
            handle = future.result()
        except Exception as exc:
            if gen == self.active_gen:
                self.fail_goal(f'send failed: {exc}')
            return
        if gen != self.active_gen:          # superseded/cancelled meanwhile
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.fail_goal('rejected by Nav2')
            return
        self.goal_handle = handle
        self.ack_failures = 0
        if self.goal is not None:
            self.goal['accepted'] = True
        handle.get_result_async().add_done_callback(
            lambda f, g=gen: self.on_goal_result(f, g))

    def on_goal_result(self, future, gen):
        if gen != self.active_gen:
            return
        self.active_gen = None
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.fail_goal(f'result error: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_succeeded()
        else:
            self.fail_goal(f'Nav2 status {status}')

    def cancel_active(self):
        handle = self.goal_handle
        self.active_gen = None
        self.goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()

    def goal_succeeded(self):
        goal = self.goal
        self.goal = None
        if goal is not None:
            self.visited_goals.append((goal['x'], goal['y']))
            self.mark_disk(goal['x'], goal['y'], self.pf('visit_mark_radius'))
            self.register_waypoint(goal['x'], goal['y'])
        self.consecutive_failures = 0
        self.relax_level = 0
        self.retry_rounds = 0
        self.cooldown_until = self.clock_seconds() + 0.3
        self.state = self.PLAN

    def fail_goal(self, reason):
        now = self.clock_seconds()
        goal = self.goal
        self.goal = None
        self.cancel_active()
        self.cooldown_until = now + self.pf('goal_cooldown')
        self.dirty = True
        if now - self.map_resized_at < 6.0:
            # Nav2 was busy resizing its costmap: not the target's fault.
            self.get_logger().warn(
                f'Goal failed ({reason}) right after a map resize; '
                'retrying later without blacklisting')
            self.state = self.PLAN
            return
        if goal is not None:
            self.blacklist.append((goal['x'], goal['y']))
        self.consecutive_failures += 1
        self.get_logger().warn(
            f'Goal failed ({reason}); consecutive={self.consecutive_failures}')
        if self.consecutive_failures >= 3:
            self.consecutive_failures = 0
            self.start_recovery()
        else:
            self.state = self.PLAN

    # ---------------- navigation monitor ----------------
    def goal_still_useful(self, goal):
        if goal['kind'] == 'coverage':
            key = int(self.cov_keys(goal['x'], goal['y']))
            return key not in self.seen
        info = self.latest_map.info
        col = int(math.floor((goal['x'] - info.origin.position.x) / info.resolution))
        row = int(math.floor((goal['y'] - info.origin.position.y) / info.resolution))
        if not (0 <= row < info.height and 0 <= col < info.width):
            return False
        data = np.asarray(self.latest_map.data, dtype=np.int16).reshape(
            info.height, info.width)
        r0, r1 = max(0, row - 4), min(info.height, row + 5)
        c0, c1 = max(0, col - 4), min(info.width, col + 5)
        return bool((data[r0:r1, c0:c1] < 0).any())

    def monitor_goal(self, now):
        goal = self.goal
        if goal is None:
            self.state = self.PLAN
            return
        age = now - goal['t0']

        # Nav2 never answered: it is overloaded or hung. Not the goal's fault.
        if not goal.get('accepted') and age > self.pf('ack_timeout'):
            self.ack_failures += 1
            self.get_logger().warn(
                f'Nav2 did not acknowledge the goal within {age:.0f}s '
                f'(#{self.ack_failures}); Nav2 may be overloaded or hung')
            self.goal = None
            self.cancel_active()
            self.cooldown_until = now + min(15.0, 2.0 * self.ack_failures)
            self.state = self.PLAN
            return

        moved = math.hypot(self.pose[0] - self.progress_pose[0],
                           self.pose[1] - self.progress_pose[1])
        turned = abs(wrap(self.yaw - self.progress_yaw))
        if moved >= self.pf('stall_distance') or turned >= 0.35:
            self.progress_pose = self.pose
            self.progress_yaw = self.yaw
            self.progress_time = now
        elif now - self.progress_time > self.pf('goal_stall_timeout'):
            self.fail_goal('stalled')
            return

        to_goal = math.hypot(goal['x'] - self.pose[0], goal['y'] - self.pose[1])
        # Wiggling (Nav2 recoveries) is not progress: require real approach.
        if to_goal < self.best_goal_dist - 0.20:
            self.best_goal_dist = to_goal
            self.best_goal_time = now
        elif now - self.best_goal_time > self.pf('no_progress_timeout'):
            self.fail_goal('no net progress toward goal')
            return

        if age > goal['timeout']:
            self.fail_goal('timeout')
            return

        if now - self.last_goal_check >= self.pf('goal_check_period'):
            self.last_goal_check = now
            if (age > self.pf('min_commit_time') and
                    to_goal > self.pf('preempt_min_distance') and
                    self.latest_map is not None and
                    not self.goal_still_useful(goal)):
                self.get_logger().info(
                    'Goal already explored en route; replanning')
                self.goal = None
                self.cancel_active()
                self.cooldown_until = now + 0.5
                self.state = self.PLAN

    # ---------------- recovery ----------------
    def start_recovery(self):
        self.stop()
        self.state = self.RECOVER
        self.rec_t0 = self.clock_seconds()
        self.get_logger().warn(
            'Holding position before replanning; explorer recovery motion '
            'is disabled')

    def recovery_step(self, now):
        self.stop()
        if now - self.rec_t0 >= self.pf('recovery_pause_time'):
            self.state = self.PLAN

    # ---------------- main loop ----------------
    def tick(self):
        if self.state == self.FINISHED:
            return
        now = self.clock_seconds()
        pose_available = self.update_pose()
        if not pose_available:
            if self.pose_unavailable_since is None:
                self.pose_unavailable_since = now
            if (self.state != self.LOCALIZATION_HOLD and
                    self.pose is not None and
                    now - self.pose_unavailable_since >
                    self.pf('pose_timeout')):
                self.enter_localization_hold(
                    'map->base TF unavailable')
            if self.state == self.LOCALIZATION_HOLD:
                self.stop()
                self.localization_candidate = None
                self.localization_stability_anchor = None
                self.localization_stable_since = None
            return
        if self.localization_hold:
            self.localization_hold_step(now)
            return
        if self.last_scan_time is None:
            return

        scan_age = now - self.last_scan_time
        if scan_age > self.pf('scan_timeout'):
            self.get_logger().warn('LiDAR scan stale; holding',
                                   throttle_duration_sec=3.0)
            if self.state == self.RECOVER:
                self.stop()
            elif self.state == self.NAVIGATE and scan_age > 5.0:
                self.goal = None
                self.cancel_active()
                self.stop()
                self.state = self.PLAN
            return

        if self.pb('publish_speed_limit') and now - self.last_speed_pub > 2.0:
            self.last_speed_pub = now
            msg = SpeedLimit()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = False
            msg.speed_limit = self.pf('linear_speed')
            self.speed_pub.publish(msg)

        if not self.nav_client.server_is_ready():
            if self.nav_down_since is None:
                self.nav_down_since = now
                self.get_logger().error(
                    'Nav2 navigate_to_pose server LOST; pausing exploration '
                    '(the running goal is not blamed or blacklisted)')
                if self.state == self.NAVIGATE:
                    self.goal = None
                    self.cancel_active()
                    self.state = self.PLAN
                self.stop()
            down = now - self.nav_down_since
            if down > 30.0 and not self.nav_down_reported:
                self.nav_down_reported = True
                self.get_logger().error(
                    'Nav2 has been down for over 30 s. Check the launch '
                    'terminal for a crash/hang of component_container_isolated '
                    'and restart Nav2; exploration resumes automatically.')
            else:
                self.get_logger().warn(
                    f'Waiting for Nav2 ({down:.0f} s)',
                    throttle_duration_sec=15.0)
            if self.state != self.RECOVER:
                return
        elif self.nav_down_since is not None:
            self.get_logger().info(
                f'Nav2 is back after {now - self.nav_down_since:.0f} s')
            self.nav_down_since = None
            self.nav_down_reported = False
            self.cooldown_until = now + 3.0

        if self.state == self.NAVIGATE:
            self.monitor_goal(now)
        elif self.state == self.RECOVER:
            self.recovery_step(now)
        elif self.state == self.PLAN:
            if now - self.last_plan_time >= self.pf('planning_period'):
                self.last_plan_time = now
                self.plan_and_dispatch()

    def stop(self):
        self.cmd_pub.publish(Twist())

    def finish(self):
        self.cancel_active()
        self.stop()
        self.save_graph()
        self.state = self.FINISHED
        self.get_logger().info(
            f'Exploration complete: {self.goal_counter} goals, '
            f'{len(self.seen)} coverage cells, '
            f'{len(self.nodes)} graph nodes.')

    # ---------------- topological graph ----------------
    def register_waypoint(self, x, y):
        merge = self.pf('node_merge_radius')
        for node in self.nodes:
            if math.hypot(node['x'] - x, node['y'] - y) < merge:
                node['visits'] += 1
                self.link_node(node['id'])
                return
        node = {'id': len(self.nodes), 'x': x, 'y': y, 'visits': 1}
        self.nodes.append(node)
        self.link_node(node['id'])

    def link_node(self, node_id):
        if self.last_node_id is not None and self.last_node_id != node_id:
            edge = (self.last_node_id, node_id)
            if edge not in self.edges and edge[::-1] not in self.edges:
                self.edges.append(edge)
        self.last_node_id = node_id
        self.dirty = True

    def save_graph(self, log=True):
        path = str(self.get_parameter('graph_save_path').value)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            payload = {
                'nodes': self.nodes,
                'edges': [list(e) for e in self.edges],
                'visited_goals': [list(p) for p in self.visited_goals],
                'blacklist': [list(p) for p in self.blacklist],
                'seen_cells': sorted(self.seen),
                'coverage_cell_size': self.cov_size,
            }
            tmp = path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(payload, f)
            os.replace(tmp, path)
            self.dirty = False
            if log:
                self.get_logger().info(f'Saved exploration graph to {path}')
        except OSError as exc:
            self.get_logger().error(f'Could not save graph: {exc}')

    def periodic_save(self):
        if self.state != self.FINISHED and self.dirty:
            self.save_graph(log=False)

    def load_graph(self):
        path = str(self.get_parameter('graph_save_path').value)
        if not os.path.exists(path):
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                p = json.load(f)
            if abs(float(p.get('coverage_cell_size', self.cov_size))
                   - self.cov_size) > 1e-6:
                self.get_logger().warn('Saved coverage cell size differs; '
                                       'ignoring coverage history')
                p['seen_cells'] = []
            self.nodes = p.get('nodes', [])
            self.edges = [tuple(e) for e in p.get('edges', [])]
            self.visited_goals = [tuple(v) for v in p.get('visited_goals', [])]
            self.blacklist = [tuple(v) for v in p.get('blacklist', [])]
            self.seen = set(int(k) for k in p.get('seen_cells', []))
            self.get_logger().info(f'Resumed exploration history from {path}')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f'Could not load graph: {exc}')

    # ---------------- visualization ----------------
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        def base(ns, mid, mtype):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            return m

        nodes = base('exploration_nodes', 1, Marker.SPHERE_LIST)
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.18
        nodes.color = ColorRGBA(r=0.1, g=0.8, b=0.9, a=0.9)
        nodes.points = [Point(x=float(n['x']), y=float(n['y']), z=0.1)
                        for n in self.nodes]
        markers.markers.append(nodes)

        edges = base('exploration_edges', 2, Marker.LINE_LIST)
        edges.scale.x = 0.035
        edges.color = ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.9)
        by_id = {n['id']: n for n in self.nodes}
        for a, b in self.edges:
            if a in by_id and b in by_id:
                edges.points.append(Point(x=float(by_id[a]['x']),
                                          y=float(by_id[a]['y']), z=0.08))
                edges.points.append(Point(x=float(by_id[b]['x']),
                                          y=float(by_id[b]['y']), z=0.08))
        markers.markers.append(edges)

        bad = base('exploration_blacklist', 3, Marker.SPHERE_LIST)
        bad.scale.x = bad.scale.y = bad.scale.z = 0.12
        bad.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.8)
        bad.points = [Point(x=float(x), y=float(y), z=0.1)
                      for x, y in self.blacklist]
        markers.markers.append(bad)

        if self.goal is not None:
            cur = base('exploration_goal', 4, Marker.SPHERE)
            cur.pose.position.x = float(self.goal['x'])
            cur.pose.position.y = float(self.goal['y'])
            cur.pose.position.z = 0.15
            cur.scale.x = cur.scale.y = cur.scale.z = 0.28
            cur.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)
            markers.markers.append(cur)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TopologicalCoverageExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()