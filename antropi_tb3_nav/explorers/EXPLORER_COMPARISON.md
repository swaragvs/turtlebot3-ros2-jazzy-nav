# Explorer Comparison

## Scope

This compares the two runnable explorer implementations:

- `topological_coverage_explorer.py` — the longer implementation with Nav2
  acknowledgement, progress, map-resize, and goal-commit safeguards.
- `smart_explorer.py` — the shorter snapshot with more footprint-relaxation
  levels and less restrictive path-distance fallback.

The implementations use similar exploration architectures. The package
mapping below is checked against the repository's current `setup.py`.

## Shared design

Both implementations:

- Subscribe to `/map` and `/scan`; get the robot pose through TF in `map`.
- Inflate occupied map cells, flood-fill from the robot, and plan only to
  reachable cells.
- Prefer frontier clusters, then target LiDAR-unseen coverage cells.
- Score candidate targets mainly by path length, turn cost, and cluster size.
- Send one `NavigateToPose` goal at a time; Nav2 handles global navigation.
- Track visited/blacklisted goals, coverage cells, and a topological waypoint
  graph; save and optionally resume this history.
- Detect stale or jumping localization and hold exploration until TF stabilizes.
- Publish only zero-velocity safety stops directly to `/cmd_vel`, and can request a
  Nav2 speed limit.

Neither is a direct-drive corridor explorer. Both depend on a working map,
TF, Nav2 action server, and costmaps.

## Differences

| Area | `topological_coverage_explorer` | `smart_explorer` |
|---|---|---|
| Default minimum goal distance | `0.80 m`; near-target tier has a `0.35 m` direct-distance floor | `0.50 m`; near-target tier has a `0.35 m` direct-distance floor |
| Frontier / coverage target point | Prefers a traversable cell with extra wall clearance (`comfort_radius`, default `0.30 m`); penalizes a non-comfort cell | Uses the cluster cell nearest its centroid, without the separate comfort mask |
| Goal acknowledgement | Tracks whether Nav2 accepted the goal; cancels/retries after `ack_timeout` (default `6 s`) | No explicit acknowledgement timeout |
| Progress detection | Detects both local stall and lack of net approach to the goal (`no_progress_timeout`, default `20 s`) | Detects local stall and overall goal timeout, but not a separate no-net-approach timeout |
| Map resize | Waits `resize_settle_time` (default `3 s`) before planning; avoids blacklisting a failure soon after resize | No corresponding map-resize settle/cooldown logic |
| Goal preemption | Requires `min_commit_time` (default `4 s`) and remaining distance over `preempt_min_distance` (default `1.2 m`) before dropping an explored-en-route goal | Can drop an explored-en-route goal after `2 s` if still over `0.5 m` away |
| Nav2 server loss | Tracks and reports server loss; cancels an active goal without treating it as a target failure | Waits when the server is unavailable for planning, but has no equivalent active-goal server-loss reporting path |
| Footprint relaxation | `max_relax_level` defaults to `1` | `max_relax_level` defaults to `3` |
| Failure recovery | After 3 failures, resets the counter and waits stationary; no explorer backup/spin | After 3 failures, resets the counter, relaxes inflation, and waits stationary; no explorer backup/spin |
| Selection behavior | More conservative target choice and navigation safeguards | More permissive target fallback and more opportunities to retry with relaxed inflation |

Both near-target tiers enforce a `0.35 m` direct distance, greater than the
configured `0.25 m` Nav2 XY tolerance. Keep those settings coordinated if the
goal-checker tolerance changes.

## Which is better?

| Requirement | Better fit | Reason |
|---|---|---|
| Nav2 acknowledgement or progress stalls | Topological snapshot | Explicit acknowledgement timeout and net-approach watchdog |
| Exploration while `/map` is resizing | Topological snapshot | Settling delay and failure suppression after a resize |
| Safer clearance from walls at target cells | Topological snapshot | Comfort mask and penalty for less-clear candidates |
| Avoiding rapid goal changes | Topological snapshot | Longer goal commitment and larger preemption distance |
| Attempting tighter/narrower accessible areas | Smart Explorer | More footprint-relaxation levels and shorter target-distance tiers |
| Simpler, less guarded behavior | Smart Explorer | Fewer special-case Nav2 and map-resize controls |
| Core frontier/coverage method | Tie | Both use reachable frontiers followed by coverage gaps |

For a maze where Nav2 intermittently fails, maps resize during exploration, or
goals are frequently cancelled, the topological snapshot is the stronger
starting point. For a map where conservative inflation leaves reachable
corridors undiscovered, Smart Explorer's additional relaxation levels may help,
but should be paired with collision-safe footprint settings and a sensible
goal-checker tolerance. Neither should be considered proven without testing on
the target robot and Nav2 configuration.

## Shared limitations to address

- **Scan/pose timestamp alignment:** both snapshots use a latest pose when
  projecting scan rays into persistent coverage. They do not use TF at each
  scan's timestamp for that update. During motion, this can smear coverage
  memory. It does not itself publish or repair the TF tree.
- **Loop-closure memory rebase:** coverage keys, visited goals, blacklist
  entries, and graph coordinates are stored in map/world coordinates. Neither
  snapshot fully re-anchors all of this history after a SLAM loop-closure
  correction. Pose-jump holds do not catch every small correction.
- **TF ownership:** neither implementation publishes `map -> odom` or
  `odom -> base`; SLAM, odometry, and robot-state publishers own those
  transforms. A rotating RViz scan display requires diagnosing those publishers
  and frame settings independently of explorer goal selection.
- **Velocity command arbitration:** the explorers no longer publish recovery
  motion; Nav2 owns active navigation and its configured behavior-tree
  recoveries. Direct explorer `/cmd_vel` output is limited to zero-velocity
  safety stops.
- **Goal-checker coupling:** both offer a near-distance tier. Their minimum
  distances must be greater than the active Nav2 XY tolerance if goals should
  not be considered reached immediately.

## Current package run mapping

In this repository, [`setup.py`](../../setup.py) maps `smart_explorer` to
`frontier_coverage/smart_explorer.py` and `topological_coverage_explorer` to
[`topological_coverage/topological_coverage_explorer.py`](topological_coverage/topological_coverage_explorer.py).
These are separate console commands for separate implementations.

With the workspace sourced and SLAM/Nav2 already running:

```bash
ros2 run antropi_tb3_nav smart_explorer --ros-args -p use_sim_time:=true
```

The topological implementation is launched with:

```bash
ros2 run antropi_tb3_nav topological_coverage_explorer --ros-args -p use_sim_time:=true
```

Run only one explorer at a time.
