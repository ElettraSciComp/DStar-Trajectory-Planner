#ifndef DSTAR_GLOBAL_PLANNER_PLUGIN_H
#define DSTAR_GLOBAL_PLANNER_PLUGIN_H

#include "rclcpp/rclcpp.hpp"
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_core/global_planner.hpp>
//https://github.com/ros-planning/navigation2/blob/main/nav2_dwb_controller/README.md
// #include <base_local_planner/world_model.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <angles/angles.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "dstar.h"

#include <boost/filesystem.hpp>
// #include <jsoncpp/json/json.h>
#include <fstream>

//---------------------------
#include "nav2_util/node_utils.hpp"

namespace dstar_global_planner
{

using namespace libdstar;

class DStarGlobalPlanner: public nav2_core::GlobalPlanner //nav2_core::BaseGlobalPlanner
{
private:
    bool initialized;

    state_map* state_grid;
    dstar* generator;
    int occupancy_threshold;
    bool initial_path;

    geometry_msgs::msg::PoseStamped current_origin;
    geometry_msgs::msg::PoseStamped current_destination;
    std::list<state_point*> trajectory;

    // costmap_2d::Costmap2DROS* current_costmap;
    // costmap_2d::Costmap2D* costmap;
    // nav2_costmap_2d::Costmap2DROS* current_costmap;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> current_costmap;
    nav2_costmap_2d::Costmap2D* costmap;

    double goal_distance_threshold;
    double repulsion_gain;
    double potential_field_radius;
    double neighbor_distance_threshold;

    long width;
    long height;
    bool erosion;
    int erosion_gap;
    int cutoff_distance;

    std::vector<nav_msgs::msg::Path> ready_paths;
    std::string paths_json_filename;
    // Json::Reader json_parser;
    // Json::Value json_paths;
    bool enable_ready_paths;
    // void parse_paths_json(const std::string& filename);

public:
    // DStarGlobalPlanner();
    // virtual ~DStarGlobalPlanner();

     DStarGlobalPlanner() = default;
     ~DStarGlobalPlanner() = default;
    // DStarGlobalPlanner(std::string name, nav2_costmap_2d::Costmap2DROS* costmap_ros);
    // virtual void initialize(std::string name, nav2_costmap_2d::Costmap2DROS* costmap_ros);
    // virtual bool makePlan(const geometry_msgs::msg::PoseStamped& start,
    //                       const geometry_msgs::msg::PoseStamped& goal,
    //                       std::vector< geometry_msgs::msg::PoseStamped >& plan);
    // virtual bool makePlan(const geometry_msgs::msg::PoseStamped& start,
    //                       const geometry_msgs::msg::PoseStamped& goal,
    //                       std::vector< geometry_msgs::msg::PoseStamped >& plan,
    //                       double& cost);

    // virtual void configure();
    // virtual void cleanup() = 0;
    // virtual void activate() = 0;
    // virtual void deactivate() = 0;
    // virtual nav_msgs::msg::Path createPlan();

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;
    // Global Costmap
    nav2_costmap_2d::Costmap2D * costmap_;
    // The global frame of the costmap
    std::string global_frame_, name_;

    //------------------------------------------------
    // void configure();
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) override;
    // Method is called at when planner server enters on_configure state. 
    // Ideally this methods should perform declarations of ROS parameters and 
    // initialization of planner’s member variables.
    // This method takes 4 input params: 
    // - shared pointer to parent node,  (const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent)
    // - planner name, (std::string name)
    // - tf buffer pointer and (std::shared_ptr<tf2_ros::Buffer> tf)
    // - shared pointer to costmap. (std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    //------------------------------------------------
    // void activate();
    void activate() override;
    // Method is called when planner server enters on_activate state.
    // Ideally this method should implement operations which are
    // neccessary before planner goes to an active state.
    //------------------------------------------------
    // void cleanup();
    void cleanup() override;
    // Method is called when planner server goes to on_cleanup state.
    // Ideally this method should clean up resoures which are 
    // created for the planner.
    //------------------------------------------------
    // void deactivate();
    void deactivate() override;
    // Method is called when planner server enters on_deactivate state. 
    // Ideally this method should implement operations which are 
    // neccessary before planner goes to an inactive state.
    //------------------------------------------------
    // nav_msgs::msg::Path createPlan();
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal
    ) override;
    // Method is called when planner server demands a global plan for specified start and goal pose. 
    // This method returns nav_msgs::msg::Path carrying global plan. 
    // This method takes 2 input parmas: 
    // - start pose and 
    // - goal pose.

    double interpolation_resolution_;


};

}

#endif
