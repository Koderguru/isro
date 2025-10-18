# Hazard alert message for lunar habitat safety system
Header header

# Alert identification
string alert_id             # Unique alert identifier
string alert_type           # Type: "environmental", "structural", "equipment", "navigation", "communication"
string severity             # Severity: "info", "warning", "critical", "emergency"

# Alert details
string title                # Alert title
string description          # Detailed alert description
string location_description # Human-readable location description
geometry_msgs/Point location # Precise location coordinates

# Alert parameters
float32 confidence          # Alert confidence level (0-1)
string[] affected_systems   # Systems affected by the hazard
string[] recommended_actions # Recommended immediate actions

# Environmental context
float32 temperature         # Temperature at alert location
float32 pressure           # Pressure at alert location
float32 radiation_level    # Radiation level at alert location
bool atmospheric_breach    # Whether atmospheric integrity is compromised

# Alert lifecycle
time detected_time         # When hazard was first detected
time acknowledged_time     # When alert was acknowledged
time resolved_time         # When alert was resolved
string status              # Status: "active", "acknowledged", "resolved", "false_positive"

# Response information
string assigned_responder  # Person/robot assigned to respond
string[] response_actions  # Actions taken in response
string[] safety_protocols  # Safety protocols activated

# Communication
bool requires_immediate_attention # Whether immediate attention is required
string[] notification_recipients # Who should be notified
string escalation_level    # Current escalation level
