# ConflictZone.msg - Predicted intersection conflict zone information
std_msgs/Header header
string zone_id                     # e.g., "AISLE_INTERSECTION_EAST"
geometry_msgs/Point center_point   # (x, y, z) of intersection
float32 radius_meters
string occupying_robot             # Robot currently inside the zone (e.g. "bcr_bot_amr1")
string[] waiting_robots            # Robots currently yielded/waiting outside (e.g. ["bcr_bot_amr2"])
string granted_robot               # Robot with right-of-way
