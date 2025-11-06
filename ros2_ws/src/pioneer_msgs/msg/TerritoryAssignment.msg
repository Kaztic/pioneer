# Territory assignment for exploration coordination
# Standard ROS2 header
std_msgs/Header header

# Robot assigned to this territory (e.g., "robot_1", "robot_2")
string assigned_robot_id

# Polygon defining the assigned territory boundaries
# Points should be in world coordinates
geometry_msgs/Polygon assigned_region

# Priority level (higher = more important)
# Used for re-allocation decisions
int32 priority

# Timestamp when assignment was made
builtin_interfaces/Time assignment_time

# Entity that made the assignment (e.g., "mission_coordinator", "consensus", "manual")
string assigned_by


