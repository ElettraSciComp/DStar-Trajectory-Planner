#ifndef DSTAR_GLOBAL_PLANNER_PLUGIN_H
#define DSTAR_GLOBAL_PLANNER_PLUGIN_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>

#include "dstar.h"

#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <fstream>

namespace dstar_global_planner
{

using namespace libdstar;

class DStarGlobalPlanner: public nav_core::BaseGlobalPlanner
{
private:
    bool initialized;

    state_map* state_grid;
    dstar* generator;
    int occupancy_threshold;
    bool initial_path;

    geometry_msgs::PoseStamped current_origin;
    geometry_msgs::PoseStamped current_destination;
    std::list<state_point*> trajectory;

    costmap_2d::Costmap2DROS* current_costmap;
    costmap_2d::Costmap2D* costmap;

    double goal_distance_threshold;
    double repulsion_gain;
    double potential_field_radius;
    double neighbor_distance_threshold;

    long width;
    long height;
    bool erosion;
    int erosion_gap;
    int cutoff_distance;

    std::vector<nav_msgs::Path> ready_paths;
    std::string paths_json_filename;
    Json::Reader json_parser;
    Json::Value json_paths;
    bool enable_ready_paths;
    void parse_paths_json(const std::string& filename);

public:
    DStarGlobalPlanner();
    virtual ~DStarGlobalPlanner();
    DStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector< geometry_msgs::PoseStamped >& plan);
    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector< geometry_msgs::PoseStamped >& plan,
                          double& cost);
};

}

#endif
