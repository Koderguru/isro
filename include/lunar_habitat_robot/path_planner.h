#ifndef LUNAR_HABITAT_ROBOT_PATH_PLANNER_H
#define LUNAR_HABITAT_ROBOT_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <global_planner/planner_core.h>
#include <navfn/navfn_ros.h>
#include <lunar_habitat_robot/HazardAlert.h>

#include <vector>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <functional>

namespace lunar_habitat_robot {

struct Node {
    int32_t x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<Node> parent;
    bool is_obstacle;
    bool is_visited;
    
    Node(int32_t x = 0, int32_t y = 0) 
        : x(x), y(y), g_cost(0.0), h_cost(0.0), f_cost(0.0), 
          parent(nullptr), is_obstacle(false), is_visited(false) {}
    
    bool operator<(const Node& other) const {
        return f_cost < other.f_cost;
    }
    
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

struct PathSegment {
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped end_pose;
    double length;
    double cost;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    
    PathSegment() : length(0.0), cost(0.0) {}
};

class PathPlanner {
public:
    enum PlannerType {
        A_STAR,
        DIJKSTRA,
        RRT,
        RRT_STAR,
        PRM,
        GLOBAL_PLANNER,
        NAVFN,
        TEB,
        DWA
    };
    
    enum PathQuality {
        OPTIMAL,
        GOOD,
        ACCEPTABLE,
        POOR,
        INVALID
    };

    PathPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~PathPlanner();

    // Initialization
    bool initialize();
    void setPlannerType(PlannerType type);
    void setCostmap(const nav_msgs::OccupancyGrid& costmap);
    
    // Path planning
    bool planPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  nav_msgs::Path& path);
    
    bool planPath(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  const std::vector<geometry_msgs::PoseStamped>& waypoints,
                  nav_msgs::Path& path);
    
    // Path optimization
    bool optimizePath(nav_msgs::Path& path);
    bool smoothPath(nav_msgs::Path& path);
    bool simplifyPath(nav_msgs::Path& path);
    
    // Path validation
    bool isPathValid(const nav_msgs::Path& path) const;
    PathQuality evaluatePathQuality(const nav_msgs::Path& path) const;
    double calculatePathCost(const nav_msgs::Path& path) const;
    double calculatePathLength(const nav_msgs::Path& path) const;
    
    // Dynamic replanning
    bool replanPath(const nav_msgs::Path& current_path,
                   const sensor_msgs::LaserScan& latest_scan,
                   nav_msgs::Path& new_path);
    
    // Hazard avoidance
    void addHazard(const HazardAlert& hazard);
    void removeHazard(const std::string& hazard_id);
    void clearHazards();
    bool avoidHazards(const nav_msgs::Path& original_path, nav_msgs::Path& safe_path);
    
    // Multi-robot coordination
    bool planCoordinatedPath(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal,
                           const std::vector<nav_msgs::Path>& other_robot_paths,
                           nav_msgs::Path& path);
    
    // Utility methods
    std::vector<geometry_msgs::PoseStamped> interpolatePath(const nav_msgs::Path& path, 
                                                           double resolution = 0.1) const;
    geometry_msgs::PoseStamped getPoseAtDistance(const nav_msgs::Path& path, 
                                                double distance) const;
    double getDistanceToGoal(const nav_msgs::Path& path, 
                           const geometry_msgs::PoseStamped& current_pose) const;

private:
    // ROS components
    ros::NodeHandle& nh_;
    ros::NodeHandle& private_nh_;
    
    // Publishers
    ros::Publisher path_pub_;
    ros::Publisher debug_path_pub_;
    ros::Publisher costmap_pub_;
    
    // Costmap
    std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<costmap_2d::Costmap2D> costmap_;
    nav_msgs::OccupancyGrid current_costmap_;
    
    // Planners
    std::unique_ptr<global_planner::GlobalPlanner> global_planner_;
    std::unique_ptr<navfn::NavfnROS> navfn_planner_;
    std::unique_ptr<teb_local_planner::TebLocalPlannerROS> teb_planner_;
    std::unique_ptr<dwa_local_planner::DWAPlannerROS> dwa_planner_;
    
    // State variables
    PlannerType current_planner_type_;
    std::atomic<bool> initialized_;
    
    // Planning parameters
    double map_resolution_;
    double inflation_radius_;
    double safety_margin_;
    double max_planning_time_;
    int32_t max_iterations_;
    double goal_tolerance_;
    double path_resolution_;
    
    // Path optimization parameters
    bool enable_smoothing_;
    bool enable_simplification_;
    double smoothing_factor_;
    int32_t simplification_threshold_;
    
    // Hazard management
    std::map<std::string, HazardAlert> active_hazards_;
    double hazard_inflation_radius_;
    
    // Thread safety
    mutable std::mutex costmap_mutex_;
    mutable std::mutex hazards_mutex_;
    
    // A* planning methods
    bool planAStar(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  nav_msgs::Path& path);
    
    bool planDijkstra(const geometry_msgs::PoseStamped& start,
                     const geometry_msgs::PoseStamped& goal,
                     nav_msgs::Path& path);
    
    bool planRRT(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                nav_msgs::Path& path);
    
    bool planRRTStar(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    nav_msgs::Path& path);
    
    bool planPRM(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                nav_msgs::Path& path);
    
    // Utility methods for grid-based planners
    std::vector<std::shared_ptr<Node>> getNeighbors(const std::shared_ptr<Node>& node) const;
    double calculateHeuristic(const std::shared_ptr<Node>& node, const std::shared_ptr<Node>& goal) const;
    double calculateDistance(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const;
    bool isNodeValid(const std::shared_ptr<Node>& node) const;
    bool isLineOfSight(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) const;
    
    // Coordinate transformations
    geometry_msgs::Point worldToMap(const geometry_msgs::Point& world_point) const;
    geometry_msgs::Point mapToWorld(int32_t x, int32_t y) const;
    std::shared_ptr<Node> worldToNode(const geometry_msgs::Point& world_point) const;
    geometry_msgs::Point nodeToWorld(const std::shared_ptr<Node>& node) const;
    
    // Path processing
    nav_msgs::Path convertNodesToPath(const std::vector<std::shared_ptr<Node>>& nodes) const;
    std::vector<std::shared_ptr<Node>> convertPathToNodes(const nav_msgs::Path& path) const;
    void addOrientationToPath(nav_msgs::Path& path) const;
    
    // Path smoothing
    nav_msgs::Path smoothPath(const nav_msgs::Path& path) const;
    nav_msgs::Path simplifyPath(const nav_msgs::Path& path) const;
    bool isPathCollisionFree(const nav_msgs::Path& path) const;
    
    // Cost calculation
    double calculateCellCost(int32_t x, int32_t y) const;
    double calculatePathSegmentCost(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& end) const;
    
    // Hazard processing
    void updateCostmapWithHazards();
    double calculateHazardCost(const geometry_msgs::Point& point) const;
    bool isPointInHazard(const geometry_msgs::Point& point) const;
    
    // RRT methods
    geometry_msgs::PoseStamped sampleRandomPose() const;
    geometry_msgs::PoseStamped steer(const geometry_msgs::PoseStamped& from,
                                   const geometry_msgs::PoseStamped& to,
                                   double step_size) const;
    bool isCollisionFree(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& end) const;
    
    // PRM methods
    void buildRoadmap();
    std::vector<geometry_msgs::PoseStamped> queryRoadmap(const geometry_msgs::PoseStamped& start,
                                                        const geometry_msgs::PoseStamped& goal) const;
    bool connectNodes(const geometry_msgs::PoseStamped& node1,
                     const geometry_msgs::PoseStamped& node2) const;
    
    // Debug and visualization
    void publishDebugPath(const nav_msgs::Path& path) const;
    void publishCostmap() const;
    void visualizeHazards() const;
};

} // namespace lunar_habitat_robot

#endif // LUNAR_HABITAT_ROBOT_PATH_PLANNER_H
