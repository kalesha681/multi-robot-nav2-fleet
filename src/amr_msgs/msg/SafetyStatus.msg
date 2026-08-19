# SafetyStatus.msg - Independent safety override status
std_msgs/Header header
string robot_id
bool emergency_stop_active
float32 current_speed
float32 min_stopping_distance      # d_safe = k * v^2 + d_min
float32 closest_obstacle_distance
float32 dynamic_speed_limit
string safety_reason               # "CLEAR", "OBSTACLE_WITHIN_SAFETY_MARGIN", "SENSOR_FAULT", "HEADING_COLLISION"
