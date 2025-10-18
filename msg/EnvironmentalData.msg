# Environmental monitoring data for lunar habitat
Header header

# Atmospheric conditions
float32 temperature          # Temperature in Celsius
float32 humidity            # Humidity percentage
float32 pressure            # Atmospheric pressure in Pa
float32 oxygen_level        # O2 concentration percentage
float32 co2_level          # CO2 concentration percentage
float32 nitrogen_level     # N2 concentration percentage

# Radiation levels
float32 radiation_alpha    # Alpha radiation (μSv/h)
float32 radiation_beta     # Beta radiation (μSv/h)
float32 radiation_gamma    # Gamma radiation (μSv/h)

# Environmental hazards
bool dust_detected         # Lunar dust contamination
bool leak_detected         # Atmospheric leak detection
bool fire_detected         # Fire/smoke detection
bool chemical_hazard       # Chemical contamination

# Location information
geometry_msgs/Point location
string zone_id             # Habitat zone identifier
string timestamp           # ISO 8601 timestamp

# Data quality indicators
float32 sensor_confidence  # Overall sensor confidence (0-1)
string[] sensor_status     # Individual sensor status messages
