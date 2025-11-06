# Overall mission status for a robot
# Standard ROS2 header
std_msgs/Header header

# Robot identifier (e.g., "robot_1", "robot_2")
string robot_id

# Mission status enumeration:
# 0 = EXPLORING - actively exploring assigned territory
# 1 = NAVIGATING - moving to a goal location
# 2 = VERIFYING - navigating to verify another robot's detection
# 3 = RETURNING - returning to base or start position
# 4 = COMPLETE - mission complete or idle
uint8 status

# If object found, this contains the pose of the detected object
# Only valid when object_found is true
geometry_msgs/PoseStamped object_pose

# True if this robot has detected an object
bool object_found

# Timestamp when status was last updated
builtin_interfaces/Time status_time

# Human-readable description of current task
string current_task


