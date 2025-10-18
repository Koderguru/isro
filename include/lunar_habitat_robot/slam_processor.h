#ifndef LUNAR_HABITAT_ROBOT_SLAM_PROCESSOR_H
#define LUNAR_HABITAT_ROBOT_SLAM_PROCESSOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <laser_geometry/laser_geometry.h>

#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <map>
#include <queue>

namespace lunar_habitat_robot {

struct MapCell {
    int8_t occupancy;
    float confidence;
    int32_t hit_count;
    int32_t miss_count;
    ros::Time last_update;
    
    MapCell() : occupancy(-1), confidence(0.0f), hit_count(0), miss_count(0) {}
};

struct Particle {
    geometry_msgs::Pose pose;
    double weight;
    nav_msgs::OccupancyGrid local_map;
    
    Particle() : weight(0.0) {}
};

class SLAMProcessor {
public:
    SLAMProcessor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~SLAMProcessor();

    // Initialization and control
    bool initialize();
    void startMapping();
    void stopMapping();
    void reset();
    
    // Map management
    nav_msgs::OccupancyGrid getCurrentMap() const;
    bool saveMap(const std::string& filename);
    bool loadMap(const std::string& filename);
    void setMapOrigin(const geometry_msgs::Pose& origin);
    
    // Localization
    geometry_msgs::PoseWithCovarianceStamped getCurrentPose() const;
    bool isLocalized() const;
    float getLocalizationConfidence() const;
    
    // Processing methods
    void processLaserScan(const sensor_msgs::LaserScan& scan);
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void updatePose(const geometry_msgs::PoseWithCovarianceStamped& pose);
    
    // Map operations
    void updateMap(const sensor_msgs::LaserScan& scan, const geometry_msgs::Pose& robot_pose);
    void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const geometry_msgs::Pose& robot_pose);
    void publishMap();
    
    // Loop closure detection
    bool detectLoopClosure();
    void performLoopClosure();
    
    // Map quality assessment
    float getMapQuality() const;
    std::vector<geometry_msgs::Point> getUnmappedAreas() const;

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher map_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher particles_pub_;
    
    // TF components
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Laser geometry
    laser_geometry::LaserProjection laser_projection_;

    // State variables
    std::atomic<bool> initialized_;
    std::atomic<bool> mapping_active_;
    std::atomic<bool> localized_;
    
    // Map data
    nav_msgs::OccupancyGrid global_map_;
    std::vector<std::vector<MapCell>> map_cells_;
    nav_msgs::MapMetaData map_metadata_;
    geometry_msgs::Pose map_origin_;
    
    // Localization
    geometry_msgs::PoseWithCovarianceStamped current_pose_;
    std::vector<Particle> particles_;
    int32_t num_particles_;
    float localization_confidence_;
    
    // SLAM parameters
    float map_resolution_;
    float map_width_;
    float map_height_;
    float max_range_;
    float min_range_;
    float hit_probability_;
    float miss_probability_;
    float free_threshold_;
    float occupied_threshold_;
    float unknown_threshold_;
    
    // Loop closure parameters
    float loop_closure_threshold_;
    int32_t min_loop_closure_distance_;
    std::vector<geometry_msgs::Pose> keyframe_poses_;
    std::vector<nav_msgs::OccupancyGrid> keyframe_maps_;
    
    // Thread safety
    mutable std::mutex map_mutex_;
    mutable std::mutex pose_mutex_;
    mutable std::mutex particles_mutex_;
    
    // Processing methods
    void initializeMap();
    void initializeParticles();
    void updateParticles();
    void resampleParticles();
    void predictParticles(const geometry_msgs::Twist& motion);
    void updateParticleWeights(const sensor_msgs::LaserScan& scan);
    
    // Map operations
    void raycastUpdate(const geometry_msgs::Point& start, const geometry_msgs::Point& end);
    void updateCell(int32_t x, int32_t y, bool hit);
    MapCell& getCell(int32_t x, int32_t y);
    const MapCell& getCell(int32_t x, int32_t y) const;
    
    // Coordinate transformations
    geometry_msgs::Point worldToMap(const geometry_msgs::Point& world_point) const;
    geometry_msgs::Point mapToWorld(int32_t x, int32_t y) const;
    bool isValidMapCoordinate(int32_t x, int32_t y) const;
    
    // Utility methods
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const;
    double calculateAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const;
    geometry_msgs::Pose samplePoseAround(const geometry_msgs::Pose& center, double radius) const;
    
    // Map quality methods
    float calculateMapEntropy() const;
    int32_t countUnknownCells() const;
    int32_t countOccupiedCells() const;
    int32_t countFreeCells() const;
    
    // Loop closure methods
    float calculateMapSimilarity(const nav_msgs::OccupancyGrid& map1, 
                                const nav_msgs::OccupancyGrid& map2) const;
    geometry_msgs::Pose findBestAlignment(const nav_msgs::OccupancyGrid& map1,
                                        const nav_msgs::OccupancyGrid& map2) const;
    
    // Point cloud processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // ICP registration
    geometry_msgs::Pose performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) const;
    
    // NDT registration
    geometry_msgs::Pose performNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) const;
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_SLAM_PROCESSOR_H
