# FleetState.msg - Global multi-robot fleet status aggregation
std_msgs/Header header
amr_msgs/RobotState[] robots
uint32 active_conflicts_count
bool fleet_emergency_stop
float32 map_merge_completion_pct
