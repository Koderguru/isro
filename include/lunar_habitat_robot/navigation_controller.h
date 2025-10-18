#ifndef LUNAR_HABITAT_ROBOT_NAVIGATION_CONTROLLER_H
#define LUNAR_HABITAT_ROBOT_NAVIGATION_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <lunar_habitat_robot/NavigationStatus.h>
#include <lunar_habitat_robot/SetNavigationGoal.h>
#include <lunar_habitat_robot/HazardAlert.h>
#include <lunar_habitat_robot/PatrolAction.h>
#include <actionlib/server/simple_action_server.h>

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

namespace lunar_habitat_robot {

class NavigationController {
public:
    NavigationController(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~NavigationController();

    // Main control methods
    bool initialize();
    void run();
    void shutdown();

    // Navigation control
    bool setGoal(const geometry_msgs::PoseStamped& goal);
    bool cancelCurrentGoal();
    bool isGoalReached() const;
    bool isNavigating() const;

    // SLAM and mapping
    void startMapping();
    void stopMapping();
    bool saveMap(const std::string& filename);
    bool loadMap(const std::string& filename);

    // Patrol management
    void startPatrol(const std::vector<std::string>& zones, float speed = 0.5f);
    void stopPatrol();

    // Status and monitoring
    NavigationStatus getNavigationStatus() const;
    geometry_msgs::Pose getCurrentPose() const;
    nav_msgs::OccupancyGrid getCurrentMap() const;

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher cmd_vel_pub_;
    ros::Publisher nav_status_pub_;
    ros::Publisher hazard_alert_pub_;
    ros::Publisher map_pub_;
    
    // Subscribers
    ros::Subscriber laser_sub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber odom_sub_;
    
    // Service servers
    ros::ServiceServer set_goal_srv_;
    ros::ServiceServer get_map_srv_;
    
    // Action clients
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    
    // Action servers
    std::unique_ptr<actionlib::SimpleActionServer<PatrolAction>> patrol_action_server_;
    
    // TF components
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Laser geometry for point cloud conversion
    laser_geometry::LaserProjection laser_projection_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> mapping_active_;
    std::atomic<bool> patrol_active_;
    std::atomic<bool> goal_active_;
    
    // Current state
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_;
    nav_msgs::OccupancyGrid current_map_;
    sensor_msgs::LaserScan latest_laser_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_pointcloud_;
    
    // Navigation parameters
    std::string robot_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    float max_linear_velocity_;
    float max_angular_velocity_;
    float safety_distance_;
    float hazard_detection_threshold_;
    
    // Patrol parameters
    std::vector<std::string> patrol_zones_;
    std::vector<geometry_msgs::PoseStamped> patrol_waypoints_;
    size_t current_waypoint_index_;
    float patrol_speed_;
    
    // Thread safety
    mutable std::mutex state_mutex_;
    mutable std::mutex map_mutex_;
    
    // Callback methods
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // Service callbacks
    bool setGoalCallback(SetNavigationGoal::Request& req, SetNavigationGoal::Response& res);
    bool getMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);
    
    // Action callbacks
    void patrolGoalCallback();
    void patrolPreemptCallback();
    
    // Navigation methods
    bool planPath(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal,
                  nav_msgs::Path& path);
    bool executePath(const nav_msgs::Path& path);
    void updateNavigationStatus();
    
    // Hazard detection
    void detectHazards();
    bool checkForObstacles(const sensor_msgs::LaserScan& scan);
    bool checkForEnvironmentalHazards();
    
    // Utility methods
    geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& pose, 
                                           const std::string& target_frame);
    double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);
    bool isPoseValid(const geometry_msgs::PoseStamped& pose);
    
    // Patrol methods
    void executePatrol();
    void generatePatrolWaypoints();
    bool moveToNextWaypoint();
    
    // Map management
    void updateMap();
    void publishMap();
    
    // Status publishing
    void publishNavigationStatus();
    void publishHazardAlert(const std::string& alert_type, const std::string& description,
                           const geometry_msgs::Point& location, const std::string& severity);
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_NAVIGATION_CONTROLLER_H
