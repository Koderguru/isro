# Navigation status message for lunar habitat robot
Header header

# Robot identification
string robot_id             # Robot identifier
string robot_type           # Robot type/model

# Current position and orientation
geometry_msgs/Pose current_pose    # Current robot pose
geometry_msgs/Twist current_velocity # Current velocity
geometry_msgs/PoseStamped goal_pose # Current navigation goal

# Navigation state
string navigation_state     # State: "idle", "planning", "navigating", "recovering", "failed"
string localization_status  # Localization status: "good", "poor", "lost"
float32 localization_confidence # Localization confidence (0-1)

# Path planning information
nav_msgs/Path planned_path  # Current planned path
float32 path_length         # Total path length
float32 path_progress       # Progress along path (0-1)
int32 waypoint_index        # Current waypoint index

# Sensor status
bool lidar_operational      # LiDAR sensor status
bool camera_operational     # Camera sensor status
bool imu_operational        # IMU sensor status
bool wheel_encoders_operational # Wheel encoder status

# Environmental awareness
bool obstacles_detected     # Whether obstacles are detected
float32 obstacle_distance   # Distance to nearest obstacle
string[] detected_hazards   # List of detected hazards
float32 dust_level          # Current dust level affecting sensors

# Performance metrics
float32 navigation_speed    # Current navigation speed
float32 battery_level       # Battery level percentage
float32 temperature         # Robot temperature
float32 mission_time        # Time since mission start

# Error information
string[] active_errors      # List of active errors
string[] warnings           # List of warnings
string last_error_message   # Last error message
time last_error_time        # Time of last error

# Communication status
bool communication_active   # Whether communication is active
float32 signal_strength     # Communication signal strength
string last_heartbeat       # Last heartbeat timestamp
