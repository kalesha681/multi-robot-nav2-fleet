# Assignment acceptance matrix

This matrix is the project's definition of done.  A feature is complete only
when it is on the live ROS path and has the stated reproducible evidence.

| Assignment requirement | Planned live component | Reproducible acceptance check | Evidence |
| --- | --- | --- | --- |
| Both AMRs contribute to one global occupancy map | `map_fusion_node` | Drive both robots through distinct unknown aisles and inspect `/fleet/merged_map`. | RViz screenshot and map-topic metadata. |
| AMR-1 prioritises frontiers and reduces repeated-traversal updates | Traversal-aware selective-map policy | Repeat the same route, then expose a frontier; prove stable revisited cells are throttled and frontier cells are sent. | Unit-test output and `/fleet/amr1_selective_stats`. |
| Concurrent global goals and slope-aware routes | Nav2 plus slope cost layer | Send both missions; compare a route with a flat alternative against the ramp-only case. | Planner paths, parameters, and action results. |
| Payload-dependent smooth movement | Payload-aware velocity smoother | Toggle payload state during the same command profile. | Recorded velocity/acceleration profile. |
| Predicted-conflict avoidance and AMR-2 yielding | Traffic-control node and predicted trajectories | Send both robots toward one narrow intersection at once. | Trajectory topics plus AMR-2 controlled-stop log. |
| Independent speed-dependent stop | Safety-override node + command arbiter | Introduce obstacle at distance below `k*v^2 + d_min` during motion. | Safety log and actuator command showing zero twist. |
| LiDAR and IMU are validated before navigation consumes them | BSP-style sensor validators | Inject invalid scan and over-limit angular velocity. | Warning logs and validated-topic inspection. |
| Expandable fleet and maintainable code | Config-driven launch/classes | Add a robot entry without duplicating robot-specific logic. | Config diff, clean build/lint, refactor walkthrough. |
| Submission quality | README and screen-share | Follow README from a clean shell and run all scenarios. | Build output and final video. |

## Step 1 interface audit

Current paths observed in the repository:

```text
Gazebo scan / imu
  -> ros_gz_bridge
  -> /bcr_bot_amr{1,2}/scan and /bcr_bot_amr{1,2}/imu
  -> scan: SLAM Toolbox and Nav2 costmaps; imu: currently unused by Nav2

/bcr_bot_amr{1,2}/map
  -> map_fusion_node
  -> /fleet/merged_map (world frame)
  -> both Nav2 global costmap static layers

Nav2 controller output
  -> /bcr_bot_amr{1,2}/cmd_vel
  -> ros_gz_bridge
  -> Gazebo differential-drive system
```

The last route has no independent command arbiter today.  The target contract,
to be introduced without changing baseline behaviour in Step 2, is:

```text
Nav2 -> smoother -> traffic gate -> safety override -> /<robot>/cmd_vel -> Gazebo
```

Safety will have the final authority to publish a zero command.  Sensor
validators will publish validated scan/IMU topics; SLAM and Nav2 will be
remapped only after validators are in place.

## Ramp feasibility spike

No ramp is currently committed to the world.  The next action is a deliberate
warehouse-layout survey to select a clear, useful location and route before a
collision-enabled ramp is added.  A manual robot traversal will then gate any
slope-cost work.
