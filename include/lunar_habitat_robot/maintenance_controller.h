#ifndef LUNAR_HABITAT_ROBOT_MAINTENANCE_CONTROLLER_H
#define LUNAR_HABITAT_ROBOT_MAINTENANCE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <lunar_habitat_robot/MaintenanceTask.h>
#include <lunar_habitat_robot/MaintenanceAction.h>
#include <lunar_habitat_robot/PatrolAction.h>
#include <lunar_habitat_robot/StartMaintenance.h>
#include <lunar_habitat_robot/EnvironmentalData.h>
#include <lunar_habitat_robot/HazardAlert.h>
#include <lunar_habitat_robot/NavigationStatus.h>

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

struct MaintenanceEquipment {
    std::string equipment_id;
    std::string equipment_type;
    std::string name;
    std::string description;
    geometry_msgs::Pose location;
    std::string status;  // "operational", "maintenance_required", "failed", "unknown"
    std::string priority;  // "low", "medium", "high", "critical"
    ros::Time last_maintenance;
    ros::Time next_maintenance;
    double maintenance_interval;  // in hours
    std::vector<std::string> required_tools;
    std::vector<std::string> maintenance_procedures;
    std::map<std::string, double> health_metrics;
    
    MaintenanceEquipment() : maintenance_interval(24.0) {}
};

struct MaintenanceProcedure {
    std::string procedure_id;
    std::string procedure_name;
    std::string equipment_type;
    std::string description;
    std::vector<std::string> steps;
    std::vector<std::string> required_tools;
    std::vector<std::string> safety_requirements;
    double estimated_duration;  // in minutes
    std::string difficulty_level;  // "easy", "medium", "hard", "expert"
    std::map<std::string, std::string> parameters;
    
    MaintenanceProcedure() : estimated_duration(30.0) {}
};

struct MaintenanceSchedule {
    std::string schedule_id;
    std::string equipment_id;
    std::string procedure_id;
    ros::Time scheduled_time;
    ros::Time deadline;
    std::string priority;
    std::string assigned_robot;
    std::string status;  // "scheduled", "in_progress", "completed", "failed", "cancelled"
    std::vector<std::string> dependencies;
    std::map<std::string, std::string> constraints;
    
    MaintenanceSchedule() {}
};

struct MaintenanceResult {
    std::string task_id;
    bool success;
    std::string status_message;
    std::vector<std::string> findings;
    std::vector<std::string> recommendations;
    bool requires_followup;
    std::string followup_notes;
    double actual_duration;
    std::vector<std::string> tools_used;
    std::vector<std::string> safety_incidents;
    std::map<std::string, double> measurements;
    
    MaintenanceResult() : success(false), requires_followup(false), actual_duration(0.0) {}
};

class MaintenanceController {
public:
    enum TaskType {
        PREVENTIVE,
        CORRECTIVE,
        EMERGENCY,
        INSPECTION,
        CALIBRATION,
        CLEANING,
        REPLACEMENT,
        REPAIR,
        PATROL,
        MONITORING
    };
    
    enum TaskStatus {
        PENDING,
        SCHEDULED,
        IN_PROGRESS,
        COMPLETED,
        FAILED,
        CANCELLED,
        ON_HOLD
    };
    
    enum TaskPriority {
        LOW,
        MEDIUM,
        HIGH,
        CRITICAL,
        EMERGENCY
    };

    MaintenanceController(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~MaintenanceController();

    // Initialization and control
    bool initialize();
    void start();
    void stop();
    void reset();
    
    // Task management
    std::string createMaintenanceTask(const std::string& equipment_id, 
                                    TaskType type, 
                                    TaskPriority priority,
                                    const std::string& description = "");
    
    bool scheduleTask(const std::string& task_id, const ros::Time& scheduled_time);
    bool cancelTask(const std::string& task_id);
    bool pauseTask(const std::string& task_id);
    bool resumeTask(const std::string& task_id);
    
    // Task execution
    bool executeTask(const std::string& task_id);
    bool executeTask(const MaintenanceTask& task);
    void executeMaintenanceProcedure(const std::string& procedure_id, 
                                   const geometry_msgs::Pose& target_location);
    
    // Equipment management
    void registerEquipment(const MaintenanceEquipment& equipment);
    void unregisterEquipment(const std::string& equipment_id);
    void updateEquipmentStatus(const std::string& equipment_id, const std::string& status);
    void updateEquipmentHealth(const std::string& equipment_id, const std::map<std::string, double>& metrics);
    
    // Procedure management
    void addMaintenanceProcedure(const MaintenanceProcedure& procedure);
    void removeMaintenanceProcedure(const std::string& procedure_id);
    void updateMaintenanceProcedure(const std::string& procedure_id, const MaintenanceProcedure& procedure);
    
    // Scheduling
    void generateMaintenanceSchedule();
    void optimizeSchedule();
    std::vector<MaintenanceSchedule> getScheduledTasks(const ros::Time& start_time, const ros::Time& end_time);
    std::vector<MaintenanceSchedule> getOverdueTasks();
    std::vector<MaintenanceSchedule> getUpcomingTasks(double hours_ahead = 24.0);
    
    // Status and monitoring
    std::vector<MaintenanceTask> getActiveTasks();
    std::vector<MaintenanceTask> getCompletedTasks();
    std::vector<MaintenanceTask> getFailedTasks();
    MaintenanceTask getTask(const std::string& task_id);
    TaskStatus getTaskStatus(const std::string& task_id);
    
    // Results and reporting
    MaintenanceResult getTaskResult(const std::string& task_id);
    std::vector<MaintenanceResult> getTaskResults(const ros::Time& start_time, const ros::Time& end_time);
    void generateMaintenanceReport(const ros::Time& start_time, const ros::Time& end_time);
    void exportMaintenanceData(const std::string& filename);
    
    // Automation
    void enableAutomaticScheduling(bool enable);
    void enablePredictiveMaintenance(bool enable);
    void setMaintenanceThresholds(const std::map<std::string, double>& thresholds);
    
    // Emergency procedures
    void triggerEmergencyMaintenance(const std::string& equipment_id, const std::string& emergency_type);
    void activateEmergencyProtocol();
    void coordinateEmergencyResponse();

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher task_status_pub_;
    ros::Publisher maintenance_report_pub_;
    ros::Publisher equipment_status_pub_;
    ros::Publisher schedule_pub_;
    
    // Subscribers
    ros::Subscriber environmental_data_sub_;
    ros::Subscriber hazard_alert_sub_;
    ros::Subscriber navigation_status_sub_;
    
    // Service servers
    ros::ServiceServer start_maintenance_srv_;
    ros::ServiceServer get_task_status_srv_;
    ros::ServiceServer schedule_task_srv_;
    
    // Action servers
    std::unique_ptr<actionlib::SimpleActionServer<MaintenanceAction>> maintenance_action_server_;
    std::unique_ptr<actionlib::SimpleActionServer<PatrolAction>> patrol_action_server_;
    
    // Action clients
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    
    // Timers
    ros::Timer scheduling_timer_;
    ros::Timer monitoring_timer_;
    ros::Timer reporting_timer_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    std::atomic<bool> automatic_scheduling_enabled_;
    std::atomic<bool> predictive_maintenance_enabled_;
    
    // Task management
    std::map<std::string, MaintenanceTask> tasks_;
    std::map<std::string, MaintenanceResult> task_results_;
    std::queue<std::string> task_queue_;
    std::map<std::string, TaskStatus> task_statuses_;
    std::map<std::string, ros::Time> task_timestamps_;
    
    // Equipment management
    std::map<std::string, MaintenanceEquipment> equipment_;
    std::map<std::string, std::map<std::string, double>> equipment_health_;
    std::map<std::string, ros::Time> last_maintenance_times_;
    
    // Procedure management
    std::map<std::string, MaintenanceProcedure> procedures_;
    std::map<std::string, std::vector<std::string>> equipment_procedures_;
    
    // Scheduling
    std::map<std::string, MaintenanceSchedule> schedules_;
    std::map<std::string, std::vector<std::string>> equipment_schedules_;
    std::map<std::string, double> maintenance_thresholds_;
    
    // Automation
    std::map<std::string, double> predictive_models_;
    std::map<std::string, std::vector<double>> health_trends_;
    std::map<std::string, ros::Time> predicted_failures_;
    
    // Thread safety
    mutable std::mutex tasks_mutex_;
    mutable std::mutex equipment_mutex_;
    mutable std::mutex procedures_mutex_;
    mutable std::mutex schedules_mutex_;
    
    // Callback methods
    void environmentalDataCallback(const EnvironmentalData::ConstPtr& msg);
    void hazardAlertCallback(const HazardAlert::ConstPtr& msg);
    void navigationStatusCallback(const NavigationStatus::ConstPtr& msg);
    
    // Service callbacks
    bool startMaintenanceCallback(StartMaintenance::Request& req, StartMaintenance::Response& res);
    bool getTaskStatusCallback(StartMaintenance::Request& req, StartMaintenance::Response& res);
    bool scheduleTaskCallback(StartMaintenance::Request& req, StartMaintenance::Response& res);
    
    // Action callbacks
    void maintenanceGoalCallback();
    void maintenancePreemptCallback();
    void patrolGoalCallback();
    void patrolPreemptCallback();
    
    // Timer callbacks
    void schedulingTimerCallback(const ros::TimerEvent& event);
    void monitoringTimerCallback(const ros::TimerEvent& event);
    void reportingTimerCallback(const ros::TimerEvent& event);
    
    // Task execution methods
    void executeMaintenanceTask(const MaintenanceTask& task);
    void executePreventiveMaintenance(const MaintenanceTask& task);
    void executeCorrectiveMaintenance(const MaintenanceTask& task);
    void executeEmergencyMaintenance(const MaintenanceTask& task);
    void executeInspectionTask(const MaintenanceTask& task);
    void executeCalibrationTask(const MaintenanceTask& task);
    void executeCleaningTask(const MaintenanceTask& task);
    void executeReplacementTask(const MaintenanceTask& task);
    void executeRepairTask(const MaintenanceTask& task);
    void executePatrolTask(const MaintenanceTask& task);
    void executeMonitoringTask(const MaintenanceTask& task);
    
    // Navigation and movement
    bool navigateToLocation(const geometry_msgs::PoseStamped& target_pose);
    bool waitForNavigationCompletion();
    bool isNavigationSuccessful();
    
    // Equipment interaction
    bool performEquipmentInspection(const std::string& equipment_id);
    bool performEquipmentCalibration(const std::string& equipment_id);
    bool performEquipmentCleaning(const std::string& equipment_id);
    bool performEquipmentRepair(const std::string& equipment_id);
    bool performEquipmentReplacement(const std::string& equipment_id);
    
    // Procedure execution
    bool executeProcedureStep(const std::string& step, const std::map<std::string, std::string>& parameters);
    bool validateProcedureStep(const std::string& step);
    bool checkSafetyRequirements(const std::vector<std::string>& requirements);
    
    // Tool management
    bool acquireTool(const std::string& tool_id);
    bool releaseTool(const std::string& tool_id);
    bool isToolAvailable(const std::string& tool_id);
    std::vector<std::string> getAvailableTools();
    
    // Scheduling methods
    void generatePreventiveSchedule();
    void generateCorrectiveSchedule();
    void generateEmergencySchedule();
    void optimizeTaskSequence();
    bool isScheduleConflict(const MaintenanceSchedule& schedule);
    void resolveScheduleConflicts();
    
    // Predictive maintenance
    void updatePredictiveModels();
    void analyzeHealthTrends();
    void predictEquipmentFailures();
    void generatePredictiveAlerts();
    double calculateFailureProbability(const std::string& equipment_id);
    
    // Quality assessment
    double assessTaskQuality(const MaintenanceResult& result);
    double assessEquipmentHealth(const std::string& equipment_id);
    std::vector<std::string> identifyMaintenanceNeeds();
    std::vector<std::string> identifyCriticalEquipment();
    
    // Reporting methods
    void generateDailyReport();
    void generateWeeklyReport();
    void generateMonthlyReport();
    void generateEquipmentReport(const std::string& equipment_id);
    void generatePerformanceReport();
    
    // Utility methods
    std::string generateTaskId();
    std::string generateScheduleId();
    std::string taskTypeToString(TaskType type);
    std::string taskStatusToString(TaskStatus status);
    std::string taskPriorityToString(TaskPriority priority);
    TaskType stringToTaskType(const std::string& type_str);
    TaskStatus stringToTaskStatus(const std::string& status_str);
    TaskPriority stringToTaskPriority(const std::string& priority_str);
    
    // Publishing methods
    void publishTaskStatus();
    void publishMaintenanceReport();
    void publishEquipmentStatus();
    void publishSchedule();
    
    // Configuration methods
    void loadConfiguration();
    void saveConfiguration();
    void loadEquipment();
    void saveEquipment();
    void loadProcedures();
    void saveProcedures();
    void loadSchedules();
    void saveSchedules();
    
    // Validation methods
    bool validateTask(const MaintenanceTask& task);
    bool validateEquipment(const MaintenanceEquipment& equipment);
    bool validateProcedure(const MaintenanceProcedure& procedure);
    bool validateSchedule(const MaintenanceSchedule& schedule);
    
    // Emergency response methods
    void executeEmergencyProtocol();
    void notifyEmergencyServices();
    void coordinateEmergencyMaintenance();
    void activateSafetySystems();
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_MAINTENANCE_CONTROLLER_H
