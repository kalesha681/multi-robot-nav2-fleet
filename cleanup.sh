pkill -9 -f "gz sim" || true
pkill -9 -f "ruby.*gz" || true
pkill -9 -f "gzserver" || true
pkill -9 -f "ign gazebo" || true
pkill -9 -f "ros2 launch" || true
pkill -9 -f "parameter_bridge" || true
pkill -9 -f "ros_gz" || true
pkill -9 -f "mission_manager_node" || true
pkill -9 -f "controller_server" || true
pkill -9 -f "planner_server" || true
pkill -9 -f "smoother_server" || true
pkill -9 -f "behavior_server" || true
pkill -9 -f "bt_navigator" || true
pkill -9 -f "waypoint_follower" || true
pkill -9 -f "velocity_smoother" || true
pkill -9 -f "lifecycle_manager" || true
pkill -9 -f "slam_toolbox" || true
pkill -9 -f "async_slam_toolbox_node" || true
pkill -9 -f "map_fusion_node" || true
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "static_transform_publisher" || true
pkill -9 -f "robot_readiness_coordinator" || true
pkill -9 -f "clock_readiness_gate" || true
pkill -9 -f "slope_cost_node" || true
pkill -9 -f "sensor_validator" || true
pkill -9 -f "safety_override" || true
pkill -9 -f "tf_relay" || true
pkill -9 -f "rviz2" || true
ros2 daemon stop 2>/dev/null || true
rm -rf /dev/shm/fastrtps* /dev/shm/sem.fastrtps* /dev/shm/*port* /dev/shm/sem.* 2>/dev/null || true
echo "Cleanup complete."

