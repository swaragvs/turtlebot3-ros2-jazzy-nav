# Smart Frontier and Coverage Explorer

## Architecture and approach

`smart_explorer.py` is the package's active Nav2-based exploration node. It
builds a robot-inflated traversability grid from `/map`, flood-fills reachable
cells from the robot pose, and selects frontier clusters before LiDAR-unseen
coverage gaps. It sends one `NavigateToPose` goal at a time, drops goals that
become explored en route, and uses a staged failure/recovery ladder. It stores
visited goals, seen cells, and a topological waypoint graph in JSON, and
publishes graph markers for visualization.

The node reads pose from `map -> base_footprint` TF (with `base_link` fallback).
If TF is stale, non-finite, or jumps beyond configured thresholds, it cancels
the active goal and waits for a stable pose before resuming. This protects
explorer decisions; it does not repair a distorted SLAM map.

## Runtime requirements

- ROS 2 Jazzy with `rclpy`, `numpy`, `nav2_msgs`, `nav_msgs`, `sensor_msgs`,
  `tf2_ros`, `visualization_msgs`, and `action_msgs`.
- Live `/map` and `/scan` topics plus `map -> base_footprint` (or `base_link`)
  TF.
- Active Nav2 `navigate_to_pose` action server and costmaps/controller.
- For Gazebo, `use_sim_time:=true` and a valid `/clock`. Explorer timers,
  freshness, stalls, and goal timeouts use the ROS clock.
- RViz is optional; `/exploration_graph` markers are only for visualization.
- Writable path for `graph_save_path` if periodic persistence is enabled.

## Run

Launch SLAM and Nav2 first, then run:

```bash
ros2 run antropi_tb3_nav smart_explorer --ros-args -p use_sim_time:=true
```

## Parameters

| Parameter | Default | Purpose |
|---|---:|---|
| `base_frame` | `base_footprint` | Robot TF frame; `base_link` is the fallback. |
| `linear_speed` | `0.15 m/s` | Speed-limit request published to Nav2 when enabled. |
| `publish_speed_limit` | `true` | Publish to `/speed_limit`. |
| `robot_radius` | `0.22 m` | Waffle-sized radius used for traversability inflation. |
| `safety_margin` | `0.08 m` | Extra clearance for goal-grid inflation. |
| `minimum_goal_distance` | `0.50 m` | Preferred minimum path length to a goal. |
| `near_goal_distance` | `0.35 m` | Minimum direct distance to a candidate goal. |
| `min_frontier_cells` | `3` | Minimum connected frontier component size. |
| `turn_weight` | `0.30` | Score penalty per radian of heading change. |
| `size_weight` | `0.40` | Score bonus for larger frontier/coverage clusters. |
| `goal_blacklist_radius` | `0.50 m` | Exclusion radius around failed goals. |
| `visited_goal_radius` | `0.35 m` | Exclusion radius around completed goals. |
| `node_merge_radius` | `0.50 m` | Merge radius for saved topological waypoints. |
| `sensed_range` | `3.0 m` | Maximum LiDAR distance used to mark observed coverage. |
| `coverage_cell_size` | `0.10 m` | Resolution of persisted coverage cells. |
| `coverage_min_cells` | `6` | Minimum size of a coverage-gap cluster. |
| `visit_mark_radius` | `0.50 m` | Radius marked seen around completed goals. |
| `planning_period` | `0.30 s` | Interval between planning attempts. |
| `goal_timeout_base` | `20.0 s` | Base goal deadline, extended by estimated path length. |
| `goal_stall_timeout` | `8.0 s` | Time without sufficient movement before goal failure. |
| `stall_distance` | `0.10 m` | Movement that resets the stall timer. |
| `goal_check_period` | `1.5 s` | How often to check whether the active target remains useful. |
| `scan_timeout` | `1.0 s` | Maximum scan age before holding navigation. |
| `pose_timeout` | `1.0 s` | Maximum pose/TF age before localization hold. |
| `pose_jump_distance` | `0.75 m` | Translation threshold for a pose jump. |
| `pose_jump_angle` | `1.20 rad` | Yaw threshold for a pose jump. |
| `pose_stability_distance` | `0.20 m` | Maximum pose change allowed during stability checks. |
| `pose_stability_angle` | `0.35 rad` | Maximum yaw change allowed during stability checks. |
| `pose_stability_duration` | `1.0 s` | Stable duration before exploration resumes. |
| `max_relax_level` | `3` | Maximum footprint-inflation relaxation level. |
| `max_retry_rounds` | `2` | Retry rounds after no eligible targets remain. |
| `no_target_retry_period` | `5.0 s` | Wait before retrying when no reachable frontier or coverage target is found; the explorer does not declare completion from a temporary empty result. |
| `recovery_pause_time` | `2.0 s` | Stationary pause after repeated Nav2 failures; explorer recovery motion is disabled. |
| `use_bounds` | `false` | Clip planning grid to configured world-coordinate bounds. |
| `x_min`, `x_max` | `-7.2`, `7.2 m` | X clipping bounds when `use_bounds` is enabled. |
| `y_min`, `y_max` | `-7.2`, `7.2 m` | Y clipping bounds when `use_bounds` is enabled. |
| `resume_graph` | `false` | Load saved exploration history at startup. |
| `save_period` | `10.0 s` | Periodic persistence interval. |
| `graph_save_path` | `~/antropi_ws/coverage_graph.json` | JSON history output/input path. |

## Time behavior and resources

The explorer uses ROS time for elapsed-time checks. Set `use_sim_time:=true`
for simulation, and ensure the simulator publishes `/clock`. The saved history
is in map/world coordinates; after a substantial SLAM map correction, old
coverage history may no longer align with the corrected map. History resume is
off by default.
