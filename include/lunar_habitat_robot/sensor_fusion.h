#ifndef LUNAR_HABITAT_ROBOT_SENSOR_FUSION_H
#define LUNAR_HABITAT_ROBOT_SENSOR_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <robot_localization/ekf_localization_node.h>
#include <robot_localization/ukf_localization_node.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

namespace lunar_habitat_robot {

struct SensorData {
    ros::Time timestamp;
    std::string sensor_id;
    std::string sensor_type;
    double confidence;
    bool is_valid;
    
    SensorData() : confidence(0.0), is_valid(false) {}
    virtual ~SensorData() = default;
};

struct LaserData : public SensorData {
    sensor_msgs::LaserScan scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
    
    LaserData() : pointcloud(new pcl::PointCloud<pcl::PointXYZ>) {
        sensor_type = "laser";
    }
};

struct CameraData : public SensorData {
    sensor_msgs::Image image;
    cv::Mat cv_image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    
    CameraData() {
        sensor_type = "camera";
    }
};

struct IMUData : public SensorData {
    sensor_msgs::Imu imu;
    geometry_msgs::Vector3 linear_acceleration;
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Quaternion orientation;
    
    IMUData() {
        sensor_type = "imu";
    }
};

struct OdometryData : public SensorData {
    nav_msgs::Odometry odometry;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    
    OdometryData() {
        sensor_type = "odometry";
    }
};

struct FusedState {
    geometry_msgs::PoseWithCovarianceStamped pose;
    geometry_msgs::TwistWithCovarianceStamped twist;
    double confidence;
    ros::Time timestamp;
    std::vector<std::string> contributing_sensors;
    
    FusedState() : confidence(0.0) {}
};

class SensorFusion {
public:
    enum FusionMethod {
        EKF,        // Extended Kalman Filter
        UKF,        // Unscented Kalman Filter
        PARTICLE,   // Particle Filter
        COMPLEMENTARY, // Complementary Filter
        WEIGHTED_AVERAGE // Weighted Average
    };
    
    enum SensorType {
        LASER,
        CAMERA,
        IMU,
        ODOMETRY,
        GPS
    };

    SensorFusion(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~SensorFusion();

    // Initialization and control
    bool initialize();
    void start();
    void stop();
    void reset();
    
    // Sensor registration
    void registerLaserSensor(const std::string& topic, const std::string& frame_id);
    void registerCameraSensor(const std::string& topic, const std::string& frame_id);
    void registerIMUSensor(const std::string& topic, const std::string& frame_id);
    void registerOdometrySensor(const std::string& topic, const std::string& frame_id);
    
    // Data processing
    void processLaserData(const sensor_msgs::LaserScan& scan);
    void processCameraData(const sensor_msgs::Image& image);
    void processIMUData(const sensor_msgs::Imu& imu);
    void processOdometryData(const nav_msgs::Odometry& odom);
    
    // State estimation
    FusedState getCurrentState() const;
    geometry_msgs::PoseWithCovarianceStamped getCurrentPose() const;
    geometry_msgs::TwistWithCovarianceStamped getCurrentTwist() const;
    double getStateConfidence() const;
    
    // Sensor calibration
    void calibrateSensors();
    void updateSensorCalibration(const std::string& sensor_id, const std::map<std::string, double>& params);
    std::map<std::string, double> getSensorCalibration(const std::string& sensor_id) const;
    
    // Quality assessment
    double getSensorQuality(const std::string& sensor_id) const;
    std::vector<std::string> getActiveSensors() const;
    std::vector<std::string> getFailedSensors() const;
    
    // Data synchronization
    void enableTimeSynchronization(bool enable);
    void setTimeSyncThreshold(double threshold);
    
    // Visualization and debugging
    void publishDebugData();
    void saveSensorData(const std::string& filename);

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher fused_pose_pub_;
    ros::Publisher fused_twist_pub_;
    ros::Publisher debug_cloud_pub_;
    ros::Publisher debug_image_pub_;
    ros::Publisher sensor_status_pub_;
    
    // Subscribers
    std::map<std::string, ros::Subscriber> laser_subs_;
    std::map<std::string, ros::Subscriber> camera_subs_;
    std::map<std::string, ros::Subscriber> imu_subs_;
    std::map<std::string, ros::Subscriber> odom_subs_;
    
    // Synchronized subscribers
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laser_sync_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> camera_sync_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sync_sub_;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sync_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, 
                                                          sensor_msgs::Image, 
                                                          sensor_msgs::Imu, 
                                                          nav_msgs::Odometry> SyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // TF components
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Robot localization
    std::unique_ptr<robot_localization::EkfLocalizationNode> ekf_filter_;
    std::unique_ptr<robot_localization::UkfLocalizationNode> ukf_filter_;
    
    // Laser geometry
    laser_geometry::LaserProjection laser_projection_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    std::atomic<bool> time_sync_enabled_;
    
    // Fusion method
    FusionMethod fusion_method_;
    
    // Sensor data storage
    std::map<std::string, std::shared_ptr<LaserData>> laser_data_;
    std::map<std::string, std::shared_ptr<CameraData>> camera_data_;
    std::map<std::string, std::shared_ptr<IMUData>> imu_data_;
    std::map<std::string, std::shared_ptr<OdometryData>> odom_data_;
    
    // Fused state
    FusedState current_state_;
    
    // Sensor calibration
    std::map<std::string, std::map<std::string, double>> sensor_calibrations_;
    
    // Sensor quality metrics
    std::map<std::string, double> sensor_qualities_;
    std::map<std::string, ros::Time> last_sensor_update_;
    std::map<std::string, int32_t> sensor_failure_counts_;
    
    // Time synchronization
    double time_sync_threshold_;
    std::deque<ros::Time> sync_timestamps_;
    
    // Thread safety
    mutable std::mutex state_mutex_;
    mutable std::mutex sensor_data_mutex_;
    mutable std::mutex calibration_mutex_;
    
    // Callback methods
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const std::string& sensor_id);
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& sensor_id);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg, const std::string& sensor_id);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& sensor_id);
    
    void synchronizedCallback(const sensor_msgs::LaserScan::ConstPtr& laser,
                             const sensor_msgs::Image::ConstPtr& camera,
                             const sensor_msgs::Imu::ConstPtr& imu,
                             const nav_msgs::Odometry::ConstPtr& odom);
    
    // Fusion methods
    void performEKF();
    void performUKF();
    void performParticleFilter();
    void performComplementaryFilter();
    void performWeightedAverage();
    
    // Data processing methods
    pcl::PointCloud<pcl::PointXYZ>::Ptr processLaserScan(const sensor_msgs::LaserScan& scan);
    cv::Mat processCameraImage(const sensor_msgs::Image& image);
    void processIMUData(const sensor_msgs::Imu& imu);
    void processOdometryData(const nav_msgs::Odometry& odom);
    
    // Point cloud processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Image processing
    std::vector<cv::KeyPoint> extractKeypoints(const cv::Mat& image);
    cv::Mat extractDescriptors(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints);
    std::vector<cv::DMatch> matchDescriptors(const cv::Mat& desc1, const cv::Mat& desc2);
    
    // Feature tracking
    void trackFeatures(const cv::Mat& current_image, const cv::Mat& previous_image);
    geometry_msgs::Pose estimateVisualOdometry(const std::vector<cv::DMatch>& matches,
                                             const std::vector<cv::KeyPoint>& keypoints1,
                                             const std::vector<cv::KeyPoint>& keypoints2);
    
    // Sensor quality assessment
    double assessLaserQuality(const sensor_msgs::LaserScan& scan);
    double assessCameraQuality(const sensor_msgs::Image& image);
    double assessIMUQuality(const sensor_msgs::Imu& imu);
    double assessOdometryQuality(const nav_msgs::Odometry& odom);
    
    // Calibration methods
    void calibrateLaserSensor(const std::string& sensor_id);
    void calibrateCameraSensor(const std::string& sensor_id);
    void calibrateIMUSensor(const std::string& sensor_id);
    void calibrateOdometrySensor(const std::string& sensor_id);
    
    // Utility methods
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const std::string& target_frame);
    bool isSensorDataValid(const std::shared_ptr<SensorData>& data);
    void updateSensorQuality(const std::string& sensor_id, double quality);
    void handleSensorFailure(const std::string& sensor_id);
    
    // Time synchronization
    bool isTimeSynchronized(const std::vector<ros::Time>& timestamps);
    ros::Time findBestSyncTime(const std::vector<ros::Time>& timestamps);
    
    // Debug and visualization
    void publishDebugPointCloud();
    void publishDebugImage();
    void publishSensorStatus();
    void visualizeSensorData();
    
    // Data logging
    void logSensorData();
    void savePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename);
    void saveImage(const cv::Mat& image, const std::string& filename);
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_SENSOR_FUSION_H
