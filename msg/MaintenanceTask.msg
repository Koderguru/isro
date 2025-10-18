# Maintenance task definition for lunar habitat robot
Header header

# Task identification
string task_id              # Unique task identifier
string task_type            # Type: "patrol", "inspection", "repair", "cleaning"
string priority             # Priority: "low", "medium", "high", "critical"

# Task parameters
geometry_msgs/Pose target_pose    # Target location for task
string[] target_equipment         # Equipment to be maintained
string[] required_tools           # Tools needed for task
float32 estimated_duration        # Estimated duration in minutes

# Task status
string status               # Status: "pending", "in_progress", "completed", "failed", "cancelled"
float32 progress            # Progress percentage (0-100)
string assigned_robot       # Robot assigned to task

# Task details
string description          # Detailed task description
string[] safety_requirements # Safety requirements and protocols
string[] environmental_conditions # Required environmental conditions

# Scheduling information
time scheduled_start        # Scheduled start time
time scheduled_end          # Scheduled end time
time actual_start           # Actual start time
time actual_end             # Actual end time

# Results and reporting
string[] findings           # Task findings and observations
string[] recommendations    # Maintenance recommendations
bool requires_followup      # Whether follow-up is needed
string followup_notes       # Follow-up notes and requirements
