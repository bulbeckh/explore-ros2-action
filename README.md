# m-explore ROS2 port with action server

Fork of [robo-friends ROS2 port](https://github.com/robo-friends/m-explore-ros2) of `m-explore` package but replacing the node control (via `/explore/resume` and `/explore/status`) with an action server. Also disables auto-start of frontier exploration.

## New exploration goal handling parameters

The explorer no longer sends the raw frontier centroid directly to Nav2. It now derives a nearby free-space target from the frontier geometry and only sends a goal if that target is far enough from the robot to be useful.

- `min_goal_distance`
  Minimum distance in meters between the robot and a derived frontier goal. Goals closer than this are skipped to avoid trivial Nav2 successes with no meaningful motion.
- `frontier_goal_search_radius_cells`
  Search radius, in costmap cells, used when projecting a frontier candidate point back onto a nearby free-space cell that Nav2 can navigate to.

These parameters are defined in [`explore/config/params.yaml`](/home/henry/Documents/ros2_ws_stocktakeorchestration/src/explore-ros2-action/explore/config/params.yaml).
