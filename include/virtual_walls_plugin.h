#ifndef VIRTUAL_WALLS_PLUGIN_H
#define VIRTUAL_WALLS_PLUGIN_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/static_layer.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <jsoncpp/json/json.h>

namespace costmap_virtual_walls
{

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

class CostmapVirtualWalls : public costmap_2d::StaticLayer
{
private:
    ros::NodeHandle node;

    std::string walls_topic_name;
    ros::Subscriber walls_subscriber;
    void wall_polygon_callback(const geometry_msgs::PolygonConstPtr msg);
    bool subscribe_done;
    bool initialized;
    bool walls_available;
    bool paths_available;

    std::string global_frame_id;
    std::vector<geometry_msgs::Polygon> walls;
    std::vector< std::vector<geometry_msgs::Point32> > paths;
    int default_cost;

    double current_x;
    double current_y;
    double current_yaw;

    std::string walls_json_filename;
    Json::Reader json_parser;
    Json::Value json_walls;
    Json::Value json_paths;
    void parse_walls_json(const std::string& filename);

public:
    CostmapVirtualWalls();
    virtual ~CostmapVirtualWalls();
    virtual void onInitialize();
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                             int min_i,
                             int min_j,
                             int max_i,
                             int max_j);
};

} // namespace costmap_virtual_walls

#endif //VIRTUAL_WALLS_PLUGIN_H
