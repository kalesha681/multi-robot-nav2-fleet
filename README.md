# multi-robot-nav2-fleet

Heterogeneous multi-robot fleet navigation stack for two AMRs (ROS 2 Humble, Gazebo, Nav2) — independent SLAM, readiness-gated startup, and concurrent mission dispatch in a simulated warehouse.

This repo is being built against a Robotics Software Engineer hiring assignment (adaptive navigation + conflict-aware path planning, logistics focus). It is a work in progress; this README tracks exactly what is implemented, what is validated, and what is still outstanding.

---

## 1. What this is

Two independent AMRs — a heavier "lead/mapper" (`bcr_bot_amr1`) and a lighter, faster "scout" (`bcr_bot_amr2`) — operate in a shared Gazebo warehouse. Each robot has its own namespace, sensors, TF tree, SLAM map, and Nav2 stack. A mission manager node dispatches concurrent navigation goals to both robots.

The distinguishing engineering decisions so far:

- **Heterogeneity is expressed through physical parameters** (mass, inertia, DiffDrive accel/velocity limits), not just cosmetic differences — AMR-1 is heavier and slower, AMR-2 is lighter and faster.
- **Startup is readiness-gated, not timer-gated.** Nav2 does not activate on a fixed delay; it activates only once a shared simulation clock is valid and each robot has a fresh, valid map and a resolvable TF chain. This replaced an earlier fragile 30/35-second `TimerAction` approach.

---

## 2. Repository structure

```
Amr_ws/
├── src/amr_sim/       # Gazebo simulation package — do not modify casually
│   ├── worlds/               small_warehouse.sdf
│   ├── urdf/                 bcr_bot_amr1.xacro, bcr_bot_amr2.xacro, gz*.xacro
│   ├── models/                AWS RoboMaker warehouse assets
│   └── launch/                simulation.launch.py, spawn_robot.launch.py
│
└── src/amr_control/   # All navigation, coordination, and mission logic
    ├── amr_control/
    │   ├── mission_manager_node.py   # sends concurrent NavigateToPose goals
    │   └── readiness_nodes.py        # clock gate + per-robot map/TF coordinators
    ├── config/
    │   ├── nav2_params_amr1.yaml
    │   ├── nav2_params_amr2.yaml
    │   └── generate_params.py
    ├── launch/
    │   ├── nav2_fleet.launch.py      # main fleet entry point
    │   ├── slam_amr1.launch.py / slam_amr2.launch.py
    │   └── nav2_amr1.launch.py / nav2_amr2.launch.py
    └── test/                          # copyright, flake8, pep257 checks
```

**Working rule:** `amr_sim` is treated as stable infrastructure and is only touched deliberately (e.g. adding a ramp to the world in a future phase). All new logic lives in `amr_control`.

---

## 3. Architecture: startup sequence

```
Gazebo world starts, /clock bridge begins publishing
        │
        ▼
Shared clock-readiness gate
  - at least one /clock message received
  - a later message has a strictly greater timestamp
  - no backward time jump since gate declared ready
  - bounded startup timeout (fails loudly, not silently, if broken)
        │
        ▼
Per-robot readiness coordinator (AMR-1 and AMR-2, independent)
  - subscribes to /bcr_bot_amrX/map (transient-local QoS)
  - validates: correct frame_id, non-zero dimensions, fresh timestamp
  - validates: TF lookup bcr_bot_amrX/map → bcr_bot_amrX/base_link succeeds
        │
        ▼
Coordinator calls /bcr_bot_amrX/lifecycle_manager_navigation → STARTUP
  (Nav2 is launched with autostart:=false; nothing activates until told to)
        │
        ▼
Coordinator polls → is_active → true
        │
        ▼
Mission manager is permitted to send NavigateToPose goals
```

Why this replaced fixed timers: a fixed delay only guarantees *elapsed wall time*, not that a map exists, that TF is current, or that Nav2's lifecycle nodes have actually reached `active`. On a loaded machine the old approach could let Nav2 attempt to activate — or even accept goals — before real navigation data existed. This is documented in detail as the resolved issue for **Phase 2** below.

---

## 4. Current status against the assignment

| Phase | Area | Status | Notes |
|---|---|---|---|
| 1 | Heterogeneous fleet (two physically distinct AMRs) | ✅ Done | Separate xacros, mass/inertia, DiffDrive accel/velocity limits per robot. Zero topic cross-talk confirmed via `ros2 topic list`. |
| 2 | Nav2 + SLAM bringup, readiness-gated startup | ✅ Done | Fixed timers removed. Shared clock gate + per-robot map/TF coordinators + explicit lifecycle `STARTUP`/`is_active` implemented and validated (see §5). |
| 3 | Ramp/slope-aware costing | ⬜ Not started | Requires adding elevation to `small_warehouse.sdf` (flat by default) and a custom Nav2 costmap layer plugin penalizing slope by angle. |
| 4 | Cooperative map fusion | ✅ Done | Merged global map drives Nav2 planning. **Unvalidated risk:** Phantom obstacles from max() conflict (deferred to Phase 5). |
| 5 | Payload-aware motion smoothing | ⬜ Not started | `velocity_smoother` accel/jerk params are currently static per robot type; not yet dynamically reparametrized on a payload-state signal. **Also includes resolving the unvalidated max() conflict from Phase 4b.** |
| 6 | Conflict-aware trajectory yielding (MAPF-lite) | ⬜ Not started | No trajectory-sharing or traffic-control node yet; robots do not currently negotiate priority at shared intersections. |
| 7 | Safety override node | ⬜ Not started | No independent safety node or `twist_mux` in the loop yet; Nav2's own local planner is the only current obstacle response. |
| 8 | Sensor validation (BSP-style) | ⬜ Not started | No IMU/scan plausibility-check layer yet. |
| 9 | Scalability / config-driven fleet | ⬜ Not started | Robot identities are currently hardcoded per launch file, not driven by a single `fleet_config.yaml`. |
| 10 | Refactor, README, video | 🟡 In progress | Flake8 findings identified (37, non-functional). This README is part of that deliverable. Refactor target and video not yet done. |

---

## 5. Validated results (Phase 1–2)

```
✓ Gazebo warehouse starts
✓ Exactly two robots spawn with distinct entity names
✓ Separate odometry, LiDAR, TF, and SLAM map topics per robot
✓ Both readiness coordinators reach PREREQUISITES_READY
✓ Both Nav2 lifecycle stacks reach ACTIVE (verified via is_active, not inferred)
✓ Mission manager dispatches concurrent goals successfully
```

Latest clean mission run:

```
AMR-1: SUCCEEDED in ~7.5 s
AMR-2: SUCCEEDED in ~8.2 s
```

No Fast DDS shared-memory errors, lifecycle heartbeat resets, laser-range warnings, or SLAM queue-full drops observed in the final clean run.

**Negative test (proves the fix, not just the feature):** first valid LiDAR scan was artificially delayed past the old 30-second timer threshold.
- Old (timer-gated) behavior: Nav2 would begin lifecycle activation regardless, with `/bcr_bot_amrX/map` still absent or invalid — goal dispatch would fail downstream in planning/costmap/TF.
- New (readiness-gated) behavior: `STARTUP` is never requested until the map and TF chain are actually valid; the coordinator logs the specific unmet precondition instead of failing silently later.

### Recent robustness fixes bundled into Phase 2
- Replaced fixed 30s/35s Nav2 startup timers with event/state-based readiness gating.
- Added shared `/clock` readiness gate (guards against false "fresh" timestamps before a sim-time epoch is established).
- Added per-robot map + TF readiness coordinators.
- Explicit Nav2 `STARTUP` request and `is_active` verification (Nav2 launched with `autostart:=false`).
- Fixed Humble-compatible SLAM Toolbox LiDAR range parameter names.
- Aligned simulation odometry TF publishing to 30 Hz.
- Configured async SLAM with a bounded (one-message) queue and controlled scan intake.
- Cleared stale Fast DDS shared-memory lock files before validation runs.

### Known intentional limitation
Readiness checking is **one-shot at startup**, not continuous. If SLAM crashes mid-mission, the system does not currently detect the loss, pause dispatch, or attempt recovery. Extending the current coordinator into a continuous monitor is a deliberate, documented next step — not an oversight — because it requires answering fleet-behavior policy questions first (abort vs. hold-and-retry a goal, whether a re-established map counts as the same navigation frame, how to prevent duplicate goal dispatch). Those are being treated as requirements to define explicitly, not details to improvise inside a launch file.

---

## 5a. Validated results (Phase 4b)

```
✓ Readiness coordinators wait for /fleet/merged_map and world->base_link TF
✓ Map fusion node publishes TRANSIENT_LOCAL merged map synchronized to sim time
✓ Nav2 global costmap consumes merged map
✓ Mission manager dispatches goals successfully in 'world' frame
```

Latest mission manager demo: AMR-1 successfully completed a 4-leg corridor sweep using the `world` frame and navigating via the merged map. Note: We verified empirically that NavfnPlanner can automatically resolve local map frames to the world frame via TF without issue, but the `world` frame goals are cleaner.

### Unvalidated Correctness Risk (Deferred Phantom Obstacle Test)
Phase 4b is functionally complete, but contains an unvalidated correctness risk around conflicting multi-robot observations (the `max(v1, v2)` rule). 
- **The Risk:** A single stale occupied reading from one robot (e.g., observing a dynamic obstacle) can permanently block a legitimate corridor for the entire fleet if the other robot's free-space readings are overridden by the `max()` logic.
- **Deferral:** An empirical test to validate this was abandoned due to resource constraints. The machine's resource ceiling (running Nav2 ×2 + SLAM ×2 + map fusion + Gazebo) cannot sustain the simulation long enough to reliably execute the complex sequence of spawning a dynamic obstacle, freezing SLAM, and routing without Gazebo OOM-crashing or hanging (`Requesting list of world names` loop). 
- **Resolution Plan:** We will design a smarter consensus heuristic (e.g. decaying trust or time-based weighted averaging) natively in **Phase 5**, instead of forcing the simulator through this setup.

---

## 6. Build and run

```bash
cd ~/Amr_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_sim amr_control
source install/setup.bash
```

**Terminal 1 — fleet bringup:**
```bash
ros2 launch amr_control nav2_fleet.launch.py
```
This starts Gazebo, the clock bridge, both robot spawns, both SLAM instances, and the readiness coordinators. Nav2 activates automatically for each robot once its prerequisites are met — there is no fixed wait time to observe; watch the coordinator logs for `PREREQUISITES_READY` and `ACTIVE`.

**Terminal 2 — send missions:**
```bash
ros2 run amr_control mission_manager_node
```
Sends AMR-1 to `bcr_bot_amr1/map → (1.0, 1.0)` and AMR-2 to `bcr_bot_amr2/map → (2.0, 1.0)` concurrently.

---

## 7. Known technical debt (tracked for Phase 10)

- `amr_control/package.xml` still has placeholder `TODO` description/license metadata.
- 37 Flake8 findings across launch files, `generate_params.py`, `setup.py`, and `mission_manager_node.py` (unused imports, PEP 8 spacing, line length) — no functional test failures.
- No refactor has been implemented yet; current leading candidate is splitting `nav2_fleet.launch.py` / `spawn_robot.launch.py` into modular includes as the fleet gains more per-robot logic.
- `amr_sim` model assets (AWS RoboMaker warehouse meshes/textures) are committed directly rather than via Git LFS; acceptable at current repo size (~5 MiB) but worth revisiting if the asset set grows.

---

## 8. Next steps

Remaining phases (3–9) build on top of the now-stable Phase 1–2 foundation. Suggested sequencing rationale:

- **Phase 4 (map fusion)** is comparatively self-contained and can be tackled independently.
- **Phase 3 (ramp costing)** requires a `amr_sim` world edit first (adding elevation), so it's a distinct, deliberate side task.
- **Phases 5 and 7** (motion smoothing, safety override) both act on the same velocity-command path via `twist_mux` and are natural to design together rather than separately.
- **Phase 6** (conflict-aware yielding) depends conceptually on Phase 5's smoother being reparametrizable.
- **Phase 8** (sensor validation) is independent and can slot in wherever convenient.
- **Phase 9** (scalability/config-driven fleet) is easiest to retrofit once the per-robot node patterns from Phases 3–8 are settled, rather than generalizing prematurely.