#ifndef LUNAR_HABITAT_ROBOT_ENVIRONMENTAL_MONITOR_H
#define LUNAR_HABITAT_ROBOT_ENVIRONMENTAL_MONITOR_H

#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <lunar_habitat_robot/EnvironmentalData.h>
#include <lunar_habitat_robot/HazardAlert.h>
#include <lunar_habitat_robot/GetEnvironmentalData.h>
#include <lunar_habitat_robot/StartMaintenance.h>

#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <map>
#include <queue>
#include <deque>
#include <algorithm>
#include <functional>
#include <chrono>
#include <fstream>

namespace lunar_habitat_robot {

struct EnvironmentalSensor {
    std::string sensor_id;
    std::string sensor_type;
    std::string location;
    geometry_msgs::Point position;
    bool is_active;
    bool is_calibrated;
    double last_reading;
    ros::Time last_update;
    double confidence;
    std::map<std::string, double> calibration_params;
    
    EnvironmentalSensor() : is_active(false), is_calibrated(false), 
                           last_reading(0.0), confidence(0.0) {}
};

struct EnvironmentalThreshold {
    std::string parameter;
    double min_value;
    double max_value;
    double warning_min;
    double warning_max;
    double critical_min;
    double critical_max;
    std::string unit;
    std::string description;
    
    EnvironmentalThreshold() : min_value(0.0), max_value(0.0), 
                              warning_min(0.0), warning_max(0.0),
                              critical_min(0.0), critical_max(0.0) {}
};

struct EnvironmentalZone {
    std::string zone_id;
    std::string zone_name;
    std::string zone_type;  // "living", "working", "storage", "life_support", "emergency"
    geometry_msgs::Point center;
    geometry_msgs::Vector3 dimensions;
    std::vector<std::string> sensor_ids;
    std::map<std::string, EnvironmentalThreshold> thresholds;
    bool is_accessible;
    bool is_habitable;
    std::string status;
    
    EnvironmentalZone() : is_accessible(true), is_habitable(true) {}
};

struct EnvironmentalAlert {
    std::string alert_id;
    std::string zone_id;
    std::string parameter;
    std::string severity;  // "info", "warning", "critical", "emergency"
    std::string description;
    double current_value;
    double threshold_value;
    ros::Time detection_time;
    ros::Time acknowledged_time;
    ros::Time resolved_time;
    std::string status;  // "active", "acknowledged", "resolved"
    std::string assigned_responder;
    std::vector<std::string> recommended_actions;
    
    EnvironmentalAlert() : current_value(0.0), threshold_value(0.0) {}
};

class EnvironmentalMonitor {
public:
    enum MonitoringMode {
        CONTINUOUS,
        PERIODIC,
        ON_DEMAND,
        EMERGENCY
    };
    
    enum AlertSeverity {
        INFO,
        WARNING,
        CRITICAL,
        EMERGENCY
    };

    EnvironmentalMonitor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~EnvironmentalMonitor();

    // Initialization and control
    bool initialize();
    void start();
    void stop();
    void reset();
    
    // Monitoring control
    void setMonitoringMode(MonitoringMode mode);
    void setMonitoringInterval(double interval);
    void enableZone(const std::string& zone_id);
    void disableZone(const std::string& zone_id);
    
    // Sensor management
    void registerSensor(const EnvironmentalSensor& sensor);
    void unregisterSensor(const std::string& sensor_id);
    void calibrateSensor(const std::string& sensor_id);
    void updateSensorCalibration(const std::string& sensor_id, const std::map<std::string, double>& params);
    
    // Zone management
    void addZone(const EnvironmentalZone& zone);
    void removeZone(const std::string& zone_id);
    void updateZoneThresholds(const std::string& zone_id, const std::map<std::string, EnvironmentalThreshold>& thresholds);
    
    // Data collection
    EnvironmentalData getCurrentData(const std::string& zone_id = "");
    std::vector<EnvironmentalData> getHistoricalData(const std::string& zone_id, 
                                                    const ros::Time& start_time, 
                                                    const ros::Time& end_time);
    std::vector<EnvironmentalData> getAllZonesData();
    
    // Alert management
    std::vector<EnvironmentalAlert> getActiveAlerts();
    std::vector<EnvironmentalAlert> getAllAlerts();
    void acknowledgeAlert(const std::string& alert_id, const std::string& responder);
    void resolveAlert(const std::string& alert_id, const std::string& resolution_notes);
    void clearAlert(const std::string& alert_id);
    
    // Threshold management
    void setParameterThreshold(const std::string& parameter, double min_value, double max_value);
    void setWarningThreshold(const std::string& parameter, double warning_min, double warning_max);
    void setCriticalThreshold(const std::string& parameter, double critical_min, double critical_max);
    
    // Health assessment
    double getZoneHealth(const std::string& zone_id);
    double getOverallHabitatHealth();
    std::vector<std::string> getUnhealthyZones();
    std::vector<std::string> getCriticalZones();
    
    // Data logging and export
    void enableDataLogging(bool enable);
    void setLoggingInterval(double interval);
    void exportData(const std::string& filename, const ros::Time& start_time, const ros::Time& end_time);
    void exportAlerts(const std::string& filename);
    
    // Emergency procedures
    void triggerEmergencyProtocol();
    void activateLifeSupportOverride();
    void initiateEvacuationProtocol();
    void activateContainmentProtocol();

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher environmental_data_pub_;
    ros::Publisher hazard_alert_pub_;
    ros::Publisher zone_status_pub_;
    ros::Publisher habitat_health_pub_;
    
    // Subscribers
    ros::Subscriber temperature_sub_;
    ros::Subscriber humidity_sub_;
    ros::Subscriber pressure_sub_;
    ros::Subscriber illuminance_sub_;
    ros::Subscriber magnetic_field_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber oxygen_sub_;
    ros::Subscriber co2_sub_;
    ros::Subscriber radiation_sub_;
    ros::Subscriber dust_sub_;
    ros::Subscriber leak_sub_;
    ros::Subscriber fire_sub_;
    
    // Service servers
    ros::ServiceServer get_data_srv_;
    ros::ServiceServer start_maintenance_srv_;
    
    // Timers
    ros::Timer monitoring_timer_;
    ros::Timer alert_check_timer_;
    ros::Timer data_logging_timer_;
    ros::Timer health_assessment_timer_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    std::atomic<bool> data_logging_enabled_;
    MonitoringMode monitoring_mode_;
    double monitoring_interval_;
    double logging_interval_;
    
    // Sensor management
    std::map<std::string, EnvironmentalSensor> sensors_;
    std::map<std::string, std::deque<double>> sensor_history_;
    std::map<std::string, ros::Time> last_sensor_update_;
    
    // Zone management
    std::map<std::string, EnvironmentalZone> zones_;
    std::map<std::string, EnvironmentalData> zone_data_;
    std::map<std::string, std::deque<EnvironmentalData>> zone_history_;
    
    // Alert management
    std::map<std::string, EnvironmentalAlert> active_alerts_;
    std::map<std::string, EnvironmentalAlert> alert_history_;
    std::map<std::string, EnvironmentalThreshold> global_thresholds_;
    
    // Data storage
    std::deque<EnvironmentalData> global_data_history_;
    std::ofstream data_log_file_;
    std::ofstream alert_log_file_;
    
    // Thread safety
    mutable std::mutex sensors_mutex_;
    mutable std::mutex zones_mutex_;
    mutable std::mutex alerts_mutex_;
    mutable std::mutex data_mutex_;
    
    // Callback methods
    void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& msg);
    void humidityCallback(const sensor_msgs::RelativeHumidity::ConstPtr& msg);
    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);
    void illuminanceCallback(const sensor_msgs::Illuminance::ConstPtr& msg);
    void magneticFieldCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void oxygenCallback(const std_msgs::Float32::ConstPtr& msg);
    void co2Callback(const std_msgs::Float32::ConstPtr& msg);
    void radiationCallback(const std_msgs::Float32::ConstPtr& msg);
    void dustCallback(const std_msgs::Bool::ConstPtr& msg);
    void leakCallback(const std_msgs::Bool::ConstPtr& msg);
    void fireCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // Service callbacks
    bool getDataCallback(GetEnvironmentalData::Request& req, GetEnvironmentalData::Response& res);
    bool startMaintenanceCallback(StartMaintenance::Request& req, StartMaintenance::Response& res);
    
    // Timer callbacks
    void monitoringTimerCallback(const ros::TimerEvent& event);
    void alertCheckTimerCallback(const ros::TimerEvent& event);
    void dataLoggingTimerCallback(const ros::TimerEvent& event);
    void healthAssessmentTimerCallback(const ros::TimerEvent& event);
    
    // Data processing methods
    void processSensorData(const std::string& sensor_id, double value, const std::string& parameter);
    void updateZoneData(const std::string& zone_id);
    void aggregateZoneData();
    void validateSensorData();
    
    // Alert processing methods
    void checkThresholds();
    void checkParameterThreshold(const std::string& parameter, double value, const std::string& zone_id);
    void createAlert(const std::string& parameter, double value, const std::string& zone_id, 
                    AlertSeverity severity, const std::string& description);
    void updateAlertStatus(const std::string& alert_id, const std::string& status);
    void escalateAlert(const std::string& alert_id);
    
    // Threshold management methods
    EnvironmentalThreshold getParameterThreshold(const std::string& parameter);
    bool isValueInRange(double value, const EnvironmentalThreshold& threshold);
    AlertSeverity getSeverityLevel(double value, const EnvironmentalThreshold& threshold);
    
    // Health assessment methods
    double calculateZoneHealth(const EnvironmentalZone& zone);
    double calculateParameterHealth(const std::string& parameter, const std::vector<double>& values);
    std::vector<std::string> identifyUnhealthyZones();
    std::vector<std::string> identifyCriticalZones();
    
    // Data logging methods
    void logEnvironmentalData(const EnvironmentalData& data);
    void logAlert(const EnvironmentalAlert& alert);
    void openLogFiles();
    void closeLogFiles();
    void rotateLogFiles();
    
    // Emergency response methods
    void executeEmergencyProtocol();
    void notifyEmergencyServices();
    void activateSafetySystems();
    void coordinateEvacuation();
    
    // Utility methods
    std::string generateAlertId();
    std::string getCurrentTimestamp();
    double calculateAverage(const std::vector<double>& values);
    double calculateStandardDeviation(const std::vector<double>& values);
    bool isSensorDataValid(const std::string& sensor_id, double value);
    
    // Publishing methods
    void publishEnvironmentalData();
    void publishHazardAlert(const EnvironmentalAlert& alert);
    void publishZoneStatus();
    void publishHabitatHealth();
    
    // Configuration methods
    void loadConfiguration();
    void saveConfiguration();
    void loadThresholds();
    void saveThresholds();
    void loadZones();
    void saveZones();
    
    // Validation methods
    bool validateSensorConfiguration();
    bool validateZoneConfiguration();
    bool validateThresholdConfiguration();
    
    // Diagnostic methods
    void performSystemDiagnostics();
    void checkSensorHealth();
    void checkCommunicationHealth();
    void generateDiagnosticReport();
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_ENVIRONMENTAL_MONITOR_H
