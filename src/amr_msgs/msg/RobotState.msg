# RobotState.msg - Individual robot status within fleet
std_msgs/Header header
string robot_id               # e.g., "bcr_bot_amr1", "bcr_bot_amr2"
string role                   # e.g., "mapper_lead", "scout_follower"
uint8 priority                # Priority level: 1 = High (no yielding), 2 = Low (yields)
float32 payload_mass_kg       # Current payload mass
geometry_msgs/Pose pose       # Current pose in world frame
geometry_msgs/Twist velocity  # Current linear and angular velocity
string operational_status     # "NAVIGATING", "YIELDING", "EMERGENCY_STOP", "IDLE", "MAPPING"
string current_goal_name      # e.g., "HEAVY_STORAGE", "PACKING_BAY_4"
