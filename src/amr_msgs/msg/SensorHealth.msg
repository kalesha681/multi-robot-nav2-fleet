# SensorHealth.msg - Sensor validation and health status
std_msgs/Header header
string robot_id
bool imu_healthy
bool scan_healthy
float32 imu_ang_vel_norm           # |omega| in rad/s
float32 imu_ang_vel_limit          # Max allowed plausibility limit
float32 valid_scan_beam_ratio      # Ratio of non-NaN, non-Inf beams in range
uint32 rejected_samples_count
string status_message              # "OK", "IMU_ANGULAR_VELOCITY_OVERLIMIT", "LIDAR_INVALID_RANGE", "FRAME_DROPPED"
