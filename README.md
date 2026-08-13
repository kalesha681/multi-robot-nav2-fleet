# Multi-AMR readiness-gated navigation

`amr_control` runs two independent SLAM Toolbox and Nav2 namespaces in the
Gazebo warehouse. Fleet launch no longer uses fixed Nav2 startup delays.

## Startup contract

1. `clock_readiness_gate` waits for two `/clock` samples that prove simulated
   time is advancing, then publishes the transient-local `/fleet/clock_ready`.
2. Each `robot_readiness_coordinator` waits for a fresh namespaced occupancy
   grid with valid geometry and known cells, then verifies the transform from
   `<robot>/map` to `<robot>/base_link` at the map timestamp.
3. It calls that robot's lifecycle manager with `STARTUP` and waits for its
   `is_active` service before reporting the stack ready.

Launch the fleet with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amr_control nav2_fleet.launch.py
```

The relevant logs are prefixed with `[FLEET_CLOCK]`, `[BCR_BOT_AMR1_READY]`,
`[BCR_BOT_AMR1_NAV2]`, and corresponding AMR-2 prefixes.

## Readiness test

To demonstrate the gate, temporarily prevent or delay the lidar scan stream.
The coordinators should report `WAITING_FOR_MAP_AND_TF` and never emit
`STARTUP_REQUESTED`. Restore the scan stream and verify the sequence
`PREREQUISITES_READY` → `STARTUP_REQUESTED` → `ACTIVE`.

For comparison, the previous timer-based launch started Nav2 at 30/35 seconds
regardless of whether a map or complete TF chain existed. Under the same scan
delay, Nav2 could appear lifecycle-active while its global costmap had no map;
attempting a goal then exposes costmap, planner, or TF errors.

## Current limitation

This implementation gates startup once. It intentionally does not recover a
robot if SLAM Toolbox later crashes or its map/TF chain becomes stale. A future
continuous monitor must define the mission policy explicitly: abort the goal,
hold position, or revalidate and retry after recovery. Nav2 `is_active` only
proves lifecycle availability; individual goals may still be rejected or fail.
