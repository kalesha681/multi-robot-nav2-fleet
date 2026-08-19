# SlopeCostZone.msg - Ramp and slope geometry metadata
std_msgs/Header header
string zone_name                   # "CUSTOM_ELEVATED_RAMP"
float32 min_x
float32 max_x
float32 min_y
float32 max_y
float32 incline_angle_deg          # 10.0 degrees
float32 platform_height_m          # 0.529 meters
float32 dynamic_cost_penalty       # Additional Nav2 cost calculated from payload
