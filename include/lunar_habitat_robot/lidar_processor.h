#ifndef LUNAR_HABITAT_ROBOT_LIDAR_PROCESSOR_H
#define LUNAR_HABITAT_ROBOT_LIDAR_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lunar_habitat_robot/HazardAlert.h>

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

struct LidarPoint {
    float x, y, z;
    float intensity;
    float range;
    float angle;
    bool is_valid;
    
    LidarPoint() : x(0.0f), y(0.0f), z(0.0f), intensity(0.0f), 
                   range(0.0f), angle(0.0f), is_valid(false) {}
};

struct Obstacle {
    geometry_msgs::Point center;
    geometry_msgs::Vector3 dimensions;
    float confidence;
    std::string type;  // "wall", "object", "person", "unknown"
    std::vector<geometry_msgs::Point> boundary_points;
    ros::Time detection_time;
    
    Obstacle() : confidence(0.0f) {}
};

struct LidarCalibration {
    float range_offset;
    float angle_offset;
    float intensity_correction;
    std::vector<float> range_corrections;
    std::vector<float> angle_corrections;
    std::vector<float> intensity_corrections;
    
    LidarCalibration() : range_offset(0.0f), angle_offset(0.0f), intensity_correction(1.0f) {}
};

class LidarProcessor {
public:
    enum ProcessingMode {
        OBSTACLE_DETECTION,
        MAPPING,
        LOCALIZATION,
        HAZARD_DETECTION,
        FULL_PROCESSING
    };
    
    enum FilterType {
        VOXEL_GRID,
        STATISTICAL_OUTLIER,
        RADIUS_OUTLIER,
        PASSTHROUGH,
        CROP_BOX
    };

    LidarProcessor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~LidarProcessor();

    // Initialization and control
    bool initialize();
    void start();
    void stop();
    void reset();
    
    // Processing modes
    void setProcessingMode(ProcessingMode mode);
    ProcessingMode getProcessingMode() const;
    
    // Data processing
    void processLaserScan(const sensor_msgs::LaserScan& scan);
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertLaserScanToPointCloud(const sensor_msgs::LaserScan& scan);
    
    // Obstacle detection
    std::vector<Obstacle> detectObstacles();
    std::vector<Obstacle> detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool isObstacleInPath(const geometry_msgs::Pose& robot_pose, 
                         const geometry_msgs::Pose& goal_pose,
                         float safety_margin = 0.5f);
    
    // Hazard detection
    std::vector<HazardAlert> detectHazards();
    std::vector<HazardAlert> detectHazards(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool detectStructuralDamage();
    bool detectDustAccumulation();
    bool detectAtmosphericLeak();
    
    // Feature extraction
    std::vector<geometry_msgs::Point> extractWallFeatures();
    std::vector<geometry_msgs::Point> extractCornerFeatures();
    std::vector<geometry_msgs::Point> extractDoorFeatures();
    std::vector<geometry_msgs::Point> extractObjectFeatures();
    
    // Point cloud filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyFilters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterByRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                     float min_range, float max_range);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterByHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                      float min_height, float max_height);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Clustering and segmentation
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Calibration
    void calibrate();
    void updateCalibration(const LidarCalibration& calibration);
    LidarCalibration getCalibration() const;
    bool isCalibrated() const;
    
    // Quality assessment
    double getDataQuality() const;
    double getSensorHealth() const;
    std::vector<std::string> getDiagnostics() const;
    
    // Data access
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLatestPointCloud() const;
    sensor_msgs::LaserScan getLatestLaserScan() const;
    std::vector<Obstacle> getLatestObstacles() const;
    std::vector<HazardAlert> getLatestHazards() const;
    
    // Configuration
    void setMinRange(float min_range);
    void setMaxRange(float max_range);
    void setMinHeight(float min_height);
    void setMaxHeight(float max_height);
    void setVoxelSize(float voxel_size);
    void setOutlierThreshold(float threshold);
    void setClusterTolerance(float tolerance);
    void setMinClusterSize(int32_t min_size);
    void setMaxClusterSize(int32_t max_size);

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher hazards_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Publisher features_pub_;
    
    // TF components
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Laser geometry
    laser_geometry::LaserProjection laser_projection_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    ProcessingMode processing_mode_;
    
    // Latest data
    sensor_msgs::LaserScan latest_laser_scan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_pointcloud_;
    std::vector<Obstacle> latest_obstacles_;
    std::vector<HazardAlert> latest_hazards_;
    
    // Calibration
    LidarCalibration calibration_;
    std::atomic<bool> calibrated_;
    
    // Processing parameters
    float min_range_;
    float max_range_;
    float min_height_;
    float max_height_;
    float voxel_size_;
    float outlier_threshold_;
    float cluster_tolerance_;
    int32_t min_cluster_size_;
    int32_t max_cluster_size_;
    
    // Quality metrics
    double data_quality_;
    double sensor_health_;
    std::vector<std::string> diagnostics_;
    
    // Thread safety
    mutable std::mutex data_mutex_;
    mutable std::mutex calibration_mutex_;
    
    // Processing methods
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyVoxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyStatisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyRadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassthroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyCropBoxFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Obstacle detection methods
    std::vector<Obstacle> detectObstaclesFromClusters(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
    Obstacle createObstacleFromCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    std::string classifyObstacle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    geometry_msgs::Point calculateClusterCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    geometry_msgs::Vector3 calculateClusterDimensions(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    std::vector<geometry_msgs::Point> calculateClusterBoundary(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    
    // Hazard detection methods
    HazardAlert createHazardAlert(const std::string& type, const std::string& description,
                                 const geometry_msgs::Point& location, const std::string& severity);
    bool detectWallCracks(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool detectCeilingDamage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool detectFloorDamage(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool detectObjectDisplacement(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool detectDustParticles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Feature extraction methods
    std::vector<geometry_msgs::Point> extractWallPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<geometry_msgs::Point> extractCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<geometry_msgs::Point> extractDoorPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<geometry_msgs::Point> extractObjectPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Clustering methods
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> performEuclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> performRegionGrowing(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Segmentation methods
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentPlanesRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentLinesRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCylindersRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // Calibration methods
    void performRangeCalibration();
    void performAngleCalibration();
    void performIntensityCalibration();
    void performSpatialCalibration();
    
    // Quality assessment methods
    double assessDataQuality(const sensor_msgs::LaserScan& scan);
    double assessSensorHealth();
    void updateDiagnostics();
    
    // Utility methods
    bool isValidPoint(const pcl::PointXYZ& point);
    bool isPointInRange(const pcl::PointXYZ& point, float min_range, float max_range);
    bool isPointInHeight(const pcl::PointXYZ& point, float min_height, float max_height);
    double calculatePointDistance(const pcl::PointXYZ& point);
    double calculatePointAngle(const pcl::PointXYZ& point);
    
    // Coordinate transformations
    geometry_msgs::Point pclToGeometryPoint(const pcl::PointXYZ& pcl_point);
    pcl::PointXYZ geometryToPclPoint(const geometry_msgs::Point& geometry_point);
    
    // Publishing methods
    void publishFilteredPointCloud();
    void publishObstacles();
    void publishHazards();
    void publishDiagnostics();
    void publishFeatures();
    
    // Debug and visualization
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void visualizeObstacles(const std::vector<Obstacle>& obstacles);
    void visualizeHazards(const std::vector<HazardAlert>& hazards);
    void visualizeFeatures(const std::vector<geometry_msgs::Point>& features);
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_LIDAR_PROCESSOR_H
