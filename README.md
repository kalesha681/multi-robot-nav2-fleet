# multi-robot-nav2-fleet

Heterogeneous multi-robot fleet navigation stack for two AMRs (ROS 2 Humble, Gazebo, Nav2) — featuring independent SLAM, real-time cooperative map fusion, **MPPI (Model Predictive Path Integral) Local Control**, physics-based slope traversability costing, readiness-gated lifecycle startup, and concurrent mission dispatch in a simulated warehouse.

---

## 1. System Overview

Two physically heterogeneous AMRs operate in a shared Gazebo warehouse environment with aisles, pallet clutter, and an industrial $10^\circ$ traversable ramp:
* **AMR-1 (`bcr_bot_amr1`)**: Heavy Lead / Mapper AGV (Tare mass $50\,\text{kg}$, dynamic payload $0-20\,\text{kg}$, $v_{\max} = 0.8\,\text{m/s}$). Dispatched to Northern Heavy Storage `(-2.0, 4.8)`.
* **AMR-2 (`bcr_bot_amr2`)**: Scout / Fast AGV (Tare mass $20\,\text{kg}$, $v_{\max} = 1.2\,\text{m/s}$). Dispatched to Packing Bay 4 `(2.5, 4.5)`.

### Core Engineering Highlights
* **Model Predictive Path Integral (MPPI) Control**: Trajectory optimization running 400 candidate rollouts over a $2.5\,\text{s}$ forward horizon ($10\,\text{Hz}$ cycle) with 8 critics (`ConstraintCritic`, `ObstaclesCritic`, `CostCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`, `GoalCritic`, `GoalAngleCritic`).
* **Path-Invalidation Behavior Tree Replanning**: Uses `navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml` to eliminate start-in-lethal aborts during periodic BT replans.
* **Physics-Based Slope Traversability Costmap**: Custom `slope_cost_node` calculates power consumption cost $C_{\text{slope}} = 100 \cdot \frac{F_{\text{incline}}}{F_{\max}} \cdot \frac{m_{\text{total}}}{m_{\text{tare}}}$ for the warehouse ramp shortcut.
* **Transient-Local Cooperative Map Fusion**: Synchronized real-time fusion of AMR-1 and AMR-2 SLAM grids onto `/fleet/merged_map`.
* **Event-Driven Readiness Gating**: Shared clock synchronization gate + per-robot SLAM/TF readiness coordinators preventing premature Nav2 lifecycle bringup.

---

## 2. Package Architecture

```
AMR_ws/
├── src/
│   ├── amr_bringup/          # Central orchestrator package (fleet.launch.py, RViz)
│   ├── amr_bsp/              # Board Support Package & sensor diagnostics (IMU, LiDAR validator)
│   ├── amr_control/          # Payload motion smoother & MAPF intersection traffic manager
│   ├── amr_mapping/          # Async SLAM Toolbox instances & cooperative map fusion node
│   ├── amr_msgs/             # Custom ROS 2 interfaces (SlopeCostZone, RobotState, SetPayload, etc.)
│   ├── amr_navigation/       # Nav2 parameter stacks, MPPI controller configs, readiness nodes, slope cost node
│   └── amr_sim/              # Gazebo warehouse world with 10° traversable ramp & robot URDFs
└── docs/                     # Architecture documentation, acceptance matrix, layout diagrams
```

---

## 3. Quickstart: Build & Launch

### Prerequisites & Dependency Installation
* Ubuntu 22.04 LTS (Jammy)
* ROS 2 Humble Desktop (`ros-humble-desktop`)

To install all simulation, bridge, Nav2 MPPI, SLAM, and Python dependencies automatically:
```bash
cd ~/AMR_ws
bash install_dependencies.sh
```
*(Or install via `pip install -r requirements.txt` and `rosdep install --from-paths src --ignore-src -r -y`)*

### Build Workspace
```bash
cd ~/AMR_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### Step 1: Launch Fleet Simulation & Navigation Stack

#### Option A: Full GUI Mode (Gazebo + Multi-Robot RViz)
```bash
ros2 launch amr_bringup fleet.launch.py headless:=false use_rviz:=true
```

#### Option B: Headless Benchmark Mode (CPU-Optimized / Remote Execution)
```bash
ros2 launch amr_bringup fleet.launch.py headless:=true use_rviz:=false
```

*Wait until both robots output `[BCR_BOT_AMR1_NAV2] ACTIVE` and `[BCR_BOT_AMR2_NAV2] ACTIVE` in the console.*

---

### Step 2: Dispatch Concurrent Missions

Open a second terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/AMR_ws/install/setup.bash
ros2 run amr_navigation mission_manager_node
```

* AMR-1 will navigate from `(0.0, 0.0)` $\to$ **South Logistics Bay `(-2.0, -5.0)`** (Wide open fairway & ramp dock access).
* AMR-2 will navigate from `(2.0, 0.0)` $\to$ **South Staging Bay `(2.0, -5.0)`** (Wide open eastern corridor).

---

### Step 3: Telemetry & Diagnostic Monitoring (Optional)

Open a third terminal to inspect real-time fleet topics:
```bash
source /opt/ros/humble/setup.bash

# Monitor slope traversability cost publishing
ros2 topic echo /fleet/slope_cost

# Monitor sensor validator health
ros2 topic echo /bcr_bot_amr1/sensor_health

# Monitor AMR-1 MPPI velocity commands
ros2 topic echo /bcr_bot_amr1/cmd_vel
```

---

## 4. Hardware Acceleration & Multi-Core Recommendations

The MPPI controller simulates 400 candidate trajectory rollouts per cycle. For recording video demonstrations and low-latency benchmark tests:
* **Recommended Specs**: Multi-core CPU (e.g. Intel i7/i9 or AMD Ryzen 7/9) with dedicated GPU.
* **Recording Rosbags**:
  ```bash
  ros2 bag record -o fleet_mission_bag /tf /tf_static /bcr_bot_amr1/odom /bcr_bot_amr2/odom /bcr_bot_amr1/cmd_vel /bcr_bot_amr2/cmd_vel /fleet/merged_map
  ```
