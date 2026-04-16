# waypoint

ROS1 package (C++) for mission-layer waypoint orchestration with fixed absolute coordinates.

## What It Does

- `mission_task_node`:
  - Uses fixed absolute waypoints for Phase 1~5
  - Publishes next goal (`geometry_msgs/PoseStamped`) to a configurable topic
  - Waits for external navigation feedback (`goal_reached`)
  - Publishes phase id (`std_msgs/String`)
  - Publishes `/cmd_unblock=true` after phase-2 goal is reached
  - Subscribes `/blockornot` and `/boxcounting` for conditional branching

## Launch

```bash
roslaunch waypoint waypoint_patrol.launch
```

## Main Params

- `grid_sampler_node/spacing_m`: waypoint spacing in meters
- `grid_sampler_node/clearance_m`: minimum clearance from obstacles
- `grid_sampler_node/unknown_is_occupied`: treat unknown cells as blocked
- `grid_sampler_node/min_x,max_x,min_y,max_y`: optional map ROI for floor-level patrol
- `mission_task_node/goal_topic`: where next-goal PoseStamped is published
- `mission_task_node/goal_reached_topic`: bool feedback from your navigation/controller layer
- `mission_task_node/phase_topic`: current phase id (`"1"`~`"5"`)
- `mission_task_node/cmd_unblock_topic`: publish bool true after phase 2
- `mission_task_node/blockornot_topic`: phase-4 branch decision
- `mission_task_node/boxcounting_topic`: phase-5 early-stop decision

## Launch Example (4 Room Goals)

```bash
roslaunch waypoint waypoint_patrol.launch
