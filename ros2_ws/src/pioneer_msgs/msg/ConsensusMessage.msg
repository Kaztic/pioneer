# TCE (Tashi Consensus Engine) consensus wrapper message
# Wraps any message type for consensus validation across robots
# Standard ROS2 header
std_msgs/Header header

# Type of message being wrapped (e.g., "DetectionReport", "TerritoryAssignment")
string message_type

# Serialized original message (JSON string or base64 encoded)
# Receivers must deserialize based on message_type
string serialized_data

# List of robot IDs that have voted (e.g., ["robot_1", "robot_2"])
string[] voting_robots

# Vote values from each robot (true = approve, false = reject)
# Index corresponds to voting_robots array
bool[] votes

# Number of votes required to reach consensus (typically 4 for 5 robots)
uint8 required_consensus

# Current number of votes collected
uint8 current_votes

# True if consensus threshold has been reached
bool consensus_reached

# Deadline for voting (consensus expires if not reached by this time)
builtin_interfaces/Time vote_deadline

