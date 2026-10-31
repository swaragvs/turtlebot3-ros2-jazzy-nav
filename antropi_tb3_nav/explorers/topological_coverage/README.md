# Topological Coverage Explorer

## Architecture and approach

`topological_coverage_explorer.py` is a standalone Nav2 frontier/coverage
explorer with topological graph persistence, map-resize settling, and
navigation acknowledgement/progress checks.

The source subscribes to `/map` and `/scan`, reads `map -> base_footprint` TF
with a `base_link` fallback, inflates occupied cells, and flood-fills reachable
free space. It scores frontier clusters and LiDAR-unseen coverage clusters,
dispatches Nav2 `NavigateToPose` goals, and maintains visited/failed goal lists
and a JSON topological graph. It includes a map-resize settle interval,
Nav2-goal acknowledgement timeout, no-net-progress check, and staged recovery.
It publishes recovery and stop commands on `/cmd_vel`; Nav2 remains responsible
for goal navigation.

NavFn is configured with `allow_unknown: true` so plans can reach frontiers at
the known/unknown boundary during SLAM. Frontier goals are still selected from
reachable known-free cells. Since this planner setting is shared by static-map
and SLAM launches, verify paths in RViz before sending goals when using an
incomplete static map.

## Requirements and time

This implementation requires ROS 2 Jazzy, NumPy, `/map`, `/scan`, TF between
`map` and the robot base, the Nav2 `navigate_to_pose` action server, and a
writable graph path. RViz is optional for its `/exploration_graph` markers.
Elapsed-time checks use the node's ROS clock; simulation requires
`use_sim_time:=true` and `/clock`.

## Parameters

This implementation's parameter defaults differ slightly from
`frontier_coverage/README.md`:

| Parameter | Default | Purpose |
|---|---:|---|
| `robot_radius` | `0.22 m` | Conservative circular footprint for a Waffle-sized robot. |
| `linear_speed` | `0.15 m/s` | Requested Nav2 speed limit while exploring. |
| `minimum_goal_distance` | `0.80 m` | Preferred minimum goal path length. |
| `near_goal_distance` | `0.35 m` | Minimum direct goal distance; set above Nav2's XY goal tolerance. |
| `goal_check_period` | `2.0 s` | Active-goal usefulness check period. |
| `min_commit_time` | `4.0 s` | Minimum time before target preemption. |
| `preempt_min_distance` | `1.2 m` | Minimum remaining distance before preemption can occur. |
| `goal_cooldown` | `1.0 s` | Pause after a failed/cancelled goal. |
| `ack_timeout` | `6.0 s` | Time allowed for Nav2 to acknowledge a goal. |
| `no_progress_timeout` | `20.0 s` | Time without net goal approach before failure. |
| `resize_settle_time` | `3.0 s` | Wait after the occupancy-grid dimensions/origin change. |
| `comfort_radius` | `0.30 m` | Preferred clearance when selecting candidate cells. |
| `max_relax_level` | `1` | Maximum inflation relaxation level. |
| `no_target_retry_period` | `5.0 s` | Wait before retrying when no reachable frontier or coverage target is found; the explorer does not declare completion from a temporary empty result. |
| `recovery_pause_time` | `2.0 s` | Stationary pause after repeated Nav2 failures; explorer recovery motion is disabled. |

Other shared parameters (base frame, inflation, frontier/coverage clustering,
goal timeout, TF stability thresholds, retries, recovery, bounds, persistence)
are described in [`../frontier_coverage/README.md`](../frontier_coverage/README.md).

## Running

Launch SLAM and Nav2 first, then run this explorer:

```bash
ros2 run antropi_tb3_nav topological_coverage_explorer --ros-args -p use_sim_time:=true
```

The separate Smart Explorer implementation is launched with
`ros2 run antropi_tb3_nav smart_explorer`.
