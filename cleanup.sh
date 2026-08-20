#!/usr/bin/env bash
# ==============================================================================
# AMR Fleet Workspace — Enhanced Cleanup Script
# Kills Gazebo / ROS 2 / Nav2 / SLAM / custom fleet nodes, purges DDS shared
# memory, stops the ROS 2 daemon, and verifies cleanup completeness.
# ==============================================================================
set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" || "${1:-}" == "-n" ]]; then
    DRY_RUN=true
fi

log_info()    { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }

kill_pattern() {
    local pattern="$1"
    local label="$2"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [[ -z "$pids" ]]; then
        log_info "$label: nothing to kill"
        return 0
    fi
    if $DRY_RUN; then
        log_info "$label: would kill PIDs: $pids"
    else
        log_info "$label: killing PIDs: $pids"
        kill -9 $pids 2>/dev/null || true
    fi
}

wait_for_death() {
    local pattern="$1"
    local retries="${2:-10}"
    for i in $(seq 1 "$retries"); do
        if ! pgrep -f "$pattern" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.3
    done
    log_warn "$pattern still alive after ${retries} retries"
    return 1
}

# ---------------------------------------------------------------------------
# Gazebo / Ignition / Simulation
# ---------------------------------------------------------------------------
log_info "=== Stopping Gazebo / Ignition / Simulation ==="
kill_pattern "gz sim" "Gazebo Sim (gz)"
kill_pattern "ruby.*gz" "Gazebo Ruby transport"
kill_pattern "gzserver" "gzserver"
kill_pattern "ign gazebo" "Ignition Gazebo"
kill_pattern "gz-gui" "gz-gui"
kill_pattern "gz-web" "gz-web"
kill_pattern "gz-component" "gz-component"
kill_pattern "gz-physics" "gz-physics"
kill_pattern "gz-sensors" "gz-sensors"
kill_pattern "gz-render" "gz-render"
kill_pattern "gz-scene" "gz-scene"
kill_pattern "gz-sim-" "gz-sim-*"
kill_pattern "gz_bridge" "gz_bridge"

# ---------------------------------------------------------------------------
# ROS 2 Bridges & Parameters
# ---------------------------------------------------------------------------
log_info "=== Stopping ROS 2 Bridges ==="
kill_pattern "parameter_bridge" "ros_gz_bridge parameter_bridge"
kill_pattern "ros_gz_bridge" "ros_gz_bridge"
kill_pattern "ros_gz_sim" "ros_gz_sim"
kill_pattern "ros_gz_image" "ros_gz_image"
kill_pattern "ros_gz" "ros_gz_*"

# ---------------------------------------------------------------------------
# ROS 2 Launch & Lifecycle
# ---------------------------------------------------------------------------
log_info "=== Stopping ROS 2 Launch / Lifecycle ==="
kill_pattern "ros2 launch" "ros2 launch"
kill_pattern "ros2 run" "ros2 run"
kill_pattern "ros2 bag" "ros2 bag"
kill_pattern "lifecycle_manager" "lifecycle_manager"
kill_pattern "lifecycle_node" "lifecycle_node"

# ---------------------------------------------------------------------------
# Nav2 Core Servers
# ---------------------------------------------------------------------------
log_patterns=(
    "controller_server"
    "planner_server"
    "smoother_server"
    "behavior_server"
    "bt_navigator"
    "waypoint_follower"
    "velocity_smoother"
    "amcl"
    "map_server"
    "costmap"
)
for p in "${kill_patterns[@]}"; do
    kill_pattern "$p" "$p"
done

# ---------------------------------------------------------------------------
# SLAM / Mapping
# ---------------------------------------------------------------------------
log_info "=== Stopping SLAM / Mapping ==="
kill_pattern "slam_toolbox" "slam_toolbox"
kill_pattern "async_slam_toolbox_node" "async_slam_toolbox_node"
kill_pattern "map_fusion_node" "map_fusion_node"
kill_pattern "map_saver" "map_saver"

# ---------------------------------------------------------------------------
# Robot State & TF
# ---------------------------------------------------------------------------
log_info "=== Stopping TF / Robot State ==="
kill_pattern "robot_state_publisher" "robot_state_publisher"
kill_pattern "static_transform_publisher" "static_transform_publisher"
kill_pattern "tf_relay" "tf_relay"
kill_pattern "tf2_ros" "tf2_ros"

# ---------------------------------------------------------------------------
# Fleet Custom Nodes
# ---------------------------------------------------------------------------
log_info "=== Stopping Fleet Custom Nodes ==="
kill_pattern "mission_manager_node" "mission_manager_node"
kill_pattern "robot_readiness_coordinator" "robot_readiness_coordinator"
kill_pattern "clock_readiness_gate" "clock_readiness_gate"
kill_pattern "slope_cost_node" "slope_cost_node"
kill_pattern "sensor_validator" "sensor_validator"
kill_pattern "safety_override" "safety_override"
kill_pattern "dynamic_safety_zone" "dynamic_safety_zone"

# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------
log_info "=== Stopping RViz2 ==="
kill_pattern "rviz2" "rviz2"
kill_pattern "rviz" "rviz"

# ---------------------------------------------------------------------------
# Python / Node leftovers from workspace
# ---------------------------------------------------------------------------
log_info "=== Stopping stray Python / ROS nodes ==="
kill_pattern "python3.*amr_" "Python AMR nodes"
kill_pattern "python.*amr_" "Python AMR nodes"

# ---------------------------------------------------------------------------
# ROS 2 Daemon
# ---------------------------------------------------------------------------
log_info "=== Stopping ROS 2 Daemon ==="
ros2 daemon stop 2>/dev/null || true

# ---------------------------------------------------------------------------
# DDS Shared Memory Purge
# ---------------------------------------------------------------------------
log_info "=== Purging DDS shared memory ==="
if $DRY_RUN; then
    log_info "Dry run: would purge DDS shared memory paths"
else
    shopt -s nullglob
    rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/*fastdds* /dev/shm/sem.* /dev/shm/*port* /dev/shm/ros2* /dev/shm/cyclonedds* 2>/dev/null || true
    shopt -u nullglob
fi

# ---------------------------------------------------------------------------
# Workspace temp / log cleanup
# ---------------------------------------------------------------------------
log_info "=== Cleaning workspace temp artifacts ==="
if $DRY_RUN; then
    log_info "Dry run: would clean log/ and tmp/ directories"
else
    rm -rf "$(pwd)"/log/*.log "$(pwd)"/log/*.bag "$(pwd)"/scratch/*.pyc 2>/dev/null || true
fi

# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
log_info "=== Verification ==="
REMAINING=$(pgrep -f "gz sim\|ros2\|nav2\|slam_toolbox\|mission_manager\|safety_override\|parameter_bridge\|rviz2\|robot_state_publisher\|lifecycle_manager\|controller_server\|planner_server\|bt_navigator" 2>/dev/null || true)
if [[ -z "$REMAINING" ]]; then
    log_info "No remaining fleet / Gazebo / ROS 2 processes detected."
else
    log_warn "Remaining PIDs that may need manual attention:"
    echo "$REMAINING" | while read -r pid; do
        echo -e "  ${YELLOW}PID $pid${NC}: $(ps -p "$pid" -o comm= 2>/dev/null || echo '<unknown>')"
    done
fi

echo
log_info "Cleanup complete."
