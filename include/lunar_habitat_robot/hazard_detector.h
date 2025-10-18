#ifndef LUNAR_HABITAT_ROBOT_HAZARD_DETECTOR_H
#define LUNAR_HABITAT_ROBOT_HAZARD_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <lunar_habitat_robot/HazardAlert.h>
#include <lunar_habitat_robot/EnvironmentalData.h>

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

namespace lunar_habitat_robot {

struct HazardPattern {
    std::string pattern_id;
    std::string hazard_type;
    std::string description;
    std::map<std::string, double> parameters;
    double confidence_threshold;
    std::vector<std::string> required_sensors;
    std::vector<std::string> detection_methods;
    
    HazardPattern() : confidence_threshold(0.5) {}
};

struct HazardSignature {
    std::string signature_id;
    std::string hazard_type;
    std::map<std::string, double> sensor_values;
    std::map<std::string, cv::Mat> visual_features;
    std::vector<geometry_msgs::Point> spatial_features;
    double confidence;
    ros::Time detection_time;
    
    HazardSignature() : confidence(0.0) {}
};

struct HazardContext {
    std::string zone_id;
    geometry_msgs::Point location;
    std::string environmental_condition;
    std::vector<std::string> nearby_equipment;
    std::vector<std::string> affected_systems;
    double risk_level;
    std::vector<std::string> mitigation_strategies;
    
    HazardContext() : risk_level(0.0) {}
};

class HazardDetector {
public:
    enum HazardType {
        STRUCTURAL_DAMAGE,
        ATMOSPHERIC_LEAK,
        FIRE_SMOKE,
        CHEMICAL_CONTAMINATION,
        RADIATION_EXPOSURE,
        DUST_ACCUMULATION,
        EQUIPMENT_FAILURE,
        NAVIGATION_HAZARD,
        COMMUNICATION_LOSS,
        POWER_FAILURE,
        THERMAL_ANOMALY,
        PRESSURE_ANOMALY,
        UNKNOWN_HAZARD
    };
    
    enum DetectionMethod {
        THRESHOLD_BASED,
        PATTERN_RECOGNITION,
        MACHINE_LEARNING,
        ANOMALY_DETECTION,
        TREND_ANALYSIS,
        MULTI_SENSOR_FUSION,
        VISUAL_ANALYSIS,
        ACOUSTIC_ANALYSIS
    };
    
    enum SeverityLevel {
        INFO,
        WARNING,
        CRITICAL,
        EMERGENCY
    };

    HazardDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~HazardDetector();

    // Initialization and control
    bool initialize();
    void start();
    void stop();
    void reset();
    
    // Detection configuration
    void enableHazardType(HazardType type);
    void disableHazardType(HazardType type);
    void setDetectionMethod(HazardType type, DetectionMethod method);
    void setConfidenceThreshold(HazardType type, double threshold);
    
    // Pattern management
    void addHazardPattern(const HazardPattern& pattern);
    void removeHazardPattern(const std::string& pattern_id);
    void updateHazardPattern(const std::string& pattern_id, const HazardPattern& pattern);
    std::vector<HazardPattern> getHazardPatterns() const;
    
    // Signature management
    void addHazardSignature(const HazardSignature& signature);
    void removeHazardSignature(const std::string& signature_id);
    void updateHazardSignature(const std::string& signature_id, const HazardSignature& signature);
    std::vector<HazardSignature> getHazardSignatures() const;
    
    // Detection methods
    std::vector<HazardAlert> detectHazards();
    std::vector<HazardAlert> detectHazards(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectHazards(const sensor_msgs::LaserScan& scan);
    std::vector<HazardAlert> detectHazards(const sensor_msgs::PointCloud2& cloud);
    std::vector<HazardAlert> detectHazards(const sensor_msgs::Image& image);
    
    // Specific hazard detection
    std::vector<HazardAlert> detectStructuralDamage();
    std::vector<HazardAlert> detectAtmosphericLeak();
    std::vector<HazardAlert> detectFireSmoke();
    std::vector<HazardAlert> detectChemicalContamination();
    std::vector<HazardAlert> detectRadiationExposure();
    std::vector<HazardAlert> detectDustAccumulation();
    std::vector<HazardAlert> detectEquipmentFailure();
    std::vector<HazardAlert> detectNavigationHazards();
    std::vector<HazardAlert> detectCommunicationLoss();
    std::vector<HazardAlert> detectPowerFailure();
    std::vector<HazardAlert> detectThermalAnomaly();
    std::vector<HazardAlert> detectPressureAnomaly();
    
    // Alert management
    std::vector<HazardAlert> getActiveAlerts();
    std::vector<HazardAlert> getAlertsByType(HazardType type);
    std::vector<HazardAlert> getAlertsBySeverity(SeverityLevel severity);
    void acknowledgeAlert(const std::string& alert_id);
    void resolveAlert(const std::string& alert_id);
    void escalateAlert(const std::string& alert_id);
    
    // Risk assessment
    double calculateRiskLevel(const HazardAlert& alert);
    double calculateOverallRiskLevel();
    std::vector<std::string> getHighRiskZones();
    std::vector<std::string> getCriticalHazards();
    
    // Learning and adaptation
    void learnFromFalsePositive(const std::string& alert_id);
    void learnFromFalseNegative(const HazardAlert& missed_alert);
    void updateDetectionParameters();
    void retrainModels();
    
    // Configuration
    void setDetectionInterval(double interval);
    void setAlertRetentionTime(double time);
    void setMaxActiveAlerts(int32_t max_alerts);
    void enableAutoEscalation(bool enable);

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher hazard_alert_pub_;
    ros::Publisher risk_assessment_pub_;
    ros::Publisher detection_status_pub_;
    ros::Publisher debug_visualization_pub_;
    
    // Subscribers
    ros::Subscriber environmental_data_sub_;
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber camera_image_sub_;
    ros::Subscriber temperature_sub_;
    ros::Subscriber pressure_sub_;
    ros::Subscriber fire_sensor_sub_;
    ros::Subscriber gas_sensor_sub_;
    ros::Subscriber radiation_sensor_sub_;
    ros::Subscriber dust_sensor_sub_;
    ros::Subscriber power_sensor_sub_;
    ros::Subscriber communication_sub_;
    
    // Timers
    ros::Timer detection_timer_;
    ros::Timer risk_assessment_timer_;
    ros::Timer cleanup_timer_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    std::atomic<bool> auto_escalation_enabled_;
    
    // Detection configuration
    std::map<HazardType, bool> enabled_hazard_types_;
    std::map<HazardType, DetectionMethod> detection_methods_;
    std::map<HazardType, double> confidence_thresholds_;
    double detection_interval_;
    double alert_retention_time_;
    int32_t max_active_alerts_;
    
    // Pattern and signature storage
    std::map<std::string, HazardPattern> hazard_patterns_;
    std::map<std::string, HazardSignature> hazard_signatures_;
    std::map<HazardType, std::vector<std::string>> type_patterns_;
    std::map<HazardType, std::vector<std::string>> type_signatures_;
    
    // Alert management
    std::map<std::string, HazardAlert> active_alerts_;
    std::map<std::string, HazardAlert> alert_history_;
    std::map<std::string, ros::Time> alert_timestamps_;
    std::map<std::string, int32_t> alert_escalation_counts_;
    
    // Risk assessment
    std::map<std::string, double> zone_risk_levels_;
    std::map<HazardType, double> type_risk_weights_;
    double overall_risk_level_;
    
    // Learning data
    std::vector<HazardAlert> false_positives_;
    std::vector<HazardAlert> false_negatives_;
    std::map<std::string, double> detection_accuracy_;
    std::map<std::string, int32_t> detection_counts_;
    
    // Thread safety
    mutable std::mutex patterns_mutex_;
    mutable std::mutex signatures_mutex_;
    mutable std::mutex alerts_mutex_;
    mutable std::mutex risk_mutex_;
    
    // Callback methods
    void environmentalDataCallback(const EnvironmentalData::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void cameraImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& msg);
    void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);
    void fireSensorCallback(const std_msgs::Bool::ConstPtr& msg);
    void gasSensorCallback(const std_msgs::Float32::ConstPtr& msg);
    void radiationSensorCallback(const std_msgs::Float32::ConstPtr& msg);
    void dustSensorCallback(const std_msgs::Bool::ConstPtr& msg);
    void powerSensorCallback(const std_msgs::Float32::ConstPtr& msg);
    void communicationCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // Timer callbacks
    void detectionTimerCallback(const ros::TimerEvent& event);
    void riskAssessmentTimerCallback(const ros::TimerEvent& event);
    void cleanupTimerCallback(const ros::TimerEvent& event);
    
    // Detection methods
    std::vector<HazardAlert> performThresholdBasedDetection();
    std::vector<HazardAlert> performPatternRecognitionDetection();
    std::vector<HazardAlert> performMachineLearningDetection();
    std::vector<HazardAlert> performAnomalyDetection();
    std::vector<HazardAlert> performTrendAnalysisDetection();
    std::vector<HazardAlert> performMultiSensorFusionDetection();
    std::vector<HazardAlert> performVisualAnalysisDetection();
    std::vector<HazardAlert> performAcousticAnalysisDetection();
    
    // Specific detection algorithms
    std::vector<HazardAlert> detectStructuralDamageThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectStructuralDamageVisual(const sensor_msgs::Image& image);
    std::vector<HazardAlert> detectStructuralDamageLaser(const sensor_msgs::LaserScan& scan);
    
    std::vector<HazardAlert> detectAtmosphericLeakThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectAtmosphericLeakPressure(const sensor_msgs::FluidPressure& pressure);
    
    std::vector<HazardAlert> detectFireSmokeThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectFireSmokeVisual(const sensor_msgs::Image& image);
    std::vector<HazardAlert> detectFireSmokeSensor(const std_msgs::Bool& fire_sensor);
    
    std::vector<HazardAlert> detectChemicalContaminationThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectChemicalContaminationGas(const std_msgs::Float32& gas_sensor);
    
    std::vector<HazardAlert> detectRadiationExposureThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectRadiationExposureSensor(const std_msgs::Float32& radiation_sensor);
    
    std::vector<HazardAlert> detectDustAccumulationThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectDustAccumulationSensor(const std_msgs::Bool& dust_sensor);
    std::vector<HazardAlert> detectDustAccumulationVisual(const sensor_msgs::Image& image);
    
    std::vector<HazardAlert> detectEquipmentFailureThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectEquipmentFailurePower(const std_msgs::Float32& power_sensor);
    
    std::vector<HazardAlert> detectNavigationHazardsLaser(const sensor_msgs::LaserScan& scan);
    std::vector<HazardAlert> detectNavigationHazardsPointCloud(const sensor_msgs::PointCloud2& cloud);
    
    std::vector<HazardAlert> detectCommunicationLossSensor(const std_msgs::Bool& communication_sensor);
    
    std::vector<HazardAlert> detectPowerFailureSensor(const std_msgs::Float32& power_sensor);
    
    std::vector<HazardAlert> detectThermalAnomalyThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectThermalAnomalyTemperature(const sensor_msgs::Temperature& temperature);
    
    std::vector<HazardAlert> detectPressureAnomalyThreshold(const EnvironmentalData& env_data);
    std::vector<HazardAlert> detectPressureAnomalyPressure(const sensor_msgs::FluidPressure& pressure);
    
    // Alert creation and management
    HazardAlert createHazardAlert(HazardType type, SeverityLevel severity, 
                                 const std::string& description, const geometry_msgs::Point& location);
    void addAlert(const HazardAlert& alert);
    void removeAlert(const std::string& alert_id);
    void updateAlertStatus(const std::string& alert_id, const std::string& status);
    void cleanupOldAlerts();
    
    // Risk assessment methods
    double calculateHazardRisk(const HazardAlert& alert);
    double calculateZoneRisk(const std::string& zone_id);
    void updateOverallRiskLevel();
    void updateZoneRiskLevels();
    
    // Pattern matching methods
    bool matchPattern(const HazardPattern& pattern, const EnvironmentalData& env_data);
    bool matchSignature(const HazardSignature& signature, const std::map<std::string, double>& sensor_values);
    double calculatePatternConfidence(const HazardPattern& pattern, const EnvironmentalData& env_data);
    double calculateSignatureConfidence(const HazardSignature& signature, const std::map<std::string, double>& sensor_values);
    
    // Machine learning methods
    void trainDetectionModels();
    void updateDetectionModels();
    double predictHazardProbability(HazardType type, const std::map<std::string, double>& features);
    
    // Anomaly detection methods
    bool isAnomalousValue(const std::string& parameter, double value);
    double calculateAnomalyScore(const std::map<std::string, double>& values);
    std::vector<std::string> identifyAnomalousParameters(const EnvironmentalData& env_data);
    
    // Trend analysis methods
    bool detectTrendAnomaly(const std::string& parameter, const std::vector<double>& values);
    double calculateTrendSlope(const std::vector<double>& values);
    double calculateTrendCorrelation(const std::vector<double>& values1, const std::vector<double>& values2);
    
    // Visual analysis methods
    std::vector<cv::Rect> detectStructuralCracks(const cv::Mat& image);
    std::vector<cv::Rect> detectFireSmokeRegions(const cv::Mat& image);
    std::vector<cv::Rect> detectDustAccumulation(const cv::Mat& image);
    std::vector<cv::Rect> detectEquipmentAnomalies(const cv::Mat& image);
    
    // Utility methods
    std::string generateAlertId();
    std::string hazardTypeToString(HazardType type);
    std::string severityLevelToString(SeverityLevel severity);
    HazardType stringToHazardType(const std::string& type_str);
    SeverityLevel stringToSeverityLevel(const std::string& severity_str);
    
    // Publishing methods
    void publishHazardAlert(const HazardAlert& alert);
    void publishRiskAssessment();
    void publishDetectionStatus();
    void publishDebugVisualization();
    
    // Configuration methods
    void loadConfiguration();
    void saveConfiguration();
    void loadHazardPatterns();
    void saveHazardPatterns();
    void loadHazardSignatures();
    void saveHazardSignatures();
    
    // Learning methods
    void updateDetectionAccuracy(const std::string& method, bool correct_detection);
    void adjustConfidenceThresholds();
    void updateRiskWeights();
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_HAZARD_DETECTOR_H
