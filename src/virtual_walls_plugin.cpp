#include "virtual_walls_plugin.h"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_virtual_walls::CostmapVirtualWalls, costmap_2d::Layer)

using namespace costmap_virtual_walls;

void CostmapVirtualWalls::wall_polygon_callback(const geometry_msgs::PolygonConstPtr msg)
{
    walls.push_back(*msg);
    ROS_INFO("Adding virtual wall at origin (%f, %f), now have %lu walls", msg->points[0].x, msg->points[0].y, walls.size());
    walls_available = true;
}

void CostmapVirtualWalls::parse_walls_json(const std::string &filename)
{
    ROS_INFO("Parsing virtual walls from %s", filename.c_str());
    std::ifstream walls_file(filename);
    Json::Value json_map;
    json_parser.parse(walls_file, json_map);
    walls_file.close();
    if(json_map.type() != Json::ValueType::objectValue)
    {
        ROS_ERROR("Cannot parse virtual walls from the file: the root object is not a dictionary!");
        return;
    }
    json_walls = json_map["vwalls"];
    if(json_walls.type() != Json::ValueType::arrayValue)
    {
        ROS_ERROR("Cannot parse virtual walls from the file: the VWALLS object is not an array!");
        return;
    }
    ROS_INFO("Parsing list of virtual walls");
    for(unsigned int i = 0; i < json_walls.size(); i++)
    {
        ROS_INFO("Parsing polygon %u", i);
        Json::Value wall = json_walls[i];
        if(wall.type() != Json::ValueType::objectValue)
        {
            ROS_ERROR("Cannot parse virtual wall %u from the file: the root namespace is not an object!", i);
            return;
        }
        Json::Value wall_name = wall["name"];
        if(wall_name.type() != Json::ValueType::stringValue)
        {
            ROS_ERROR("Cannot parse virtual wall %u from the file:  the virtual wall should have a name!", i);
            return;
        }
        ROS_INFO("Parsing virtual wall %s", wall_name.asString().c_str());
        Json::Value wall_polygon = wall["polygon"];
        if(wall_polygon.type() != Json::ValueType::arrayValue)
        {
            ROS_ERROR("Cannot parse virtual wall contents:  the virtual wall convex should be defined as an array!");
            return;
        }
        if((wall_polygon.size() % 2) != 0)
        {
            ROS_ERROR("Cannot parse object: the point coordinates are not pairable");
            return;
        }
        geometry_msgs::Polygon wall_poly;
        for(unsigned int index = 0; index < wall_polygon.size(); index += 2)
        {
            ROS_INFO("Parsing point %u", index / 2);
            geometry_msgs::Point32 point;
            point.x = wall_polygon[index].asDouble();
            point.y = wall_polygon[index + 1].asDouble();
            point.z = 0;
            wall_poly.points.push_back(point);
        }
        walls.push_back(wall_poly);
    }
    walls_available = true;
    ROS_INFO("Parsing list of freepath selectors");
    json_paths = json_map["vpaths"];
    if(json_paths.type() != Json::ValueType::arrayValue)
    {
        ROS_ERROR("Cannot parse freepath selectors from the file: the VPATHS object is not an array (%i)!", json_paths.type());
        return;
    }
    for(unsigned int i = 0; i < json_paths.size(); i++)
    {
        ROS_INFO("Parsing selector %u", i);
        Json::Value path = json_paths[i];
        if(path.type() != Json::ValueType::objectValue)
        {
            ROS_ERROR("Cannot parse freepath selector %u from the file: the root namespace is not an object!", i);
            return;
        }
        Json::Value path_name = path["name"];
        if(path_name.type() != Json::ValueType::stringValue)
        {
            ROS_ERROR("Cannot parse freepath selector %u from the file:  the freepath selector should have a name!", i);
            return;
        }
        ROS_INFO("Parsing freepath selector %s", path_name.asString().c_str());
        Json::Value path_polygon = path["polygon"];
        if(path_polygon.type() != Json::ValueType::arrayValue)
        {
            ROS_ERROR("Cannot parse freepath selector contents:  the freepath selector point chain should be defined as an array!");
            return;
        }
        if((path_polygon.size() % 2) != 0)
        {
            ROS_ERROR("Cannot parse object: the point coordinates are not pairable");
            return;
        }
        std::vector<geometry_msgs::Point32> new_path;
        for(unsigned int index = 0; index < path_polygon.size(); index += 2)
        {
            ROS_INFO("Parsing point %u", index / 2);
            geometry_msgs::Point32 point;
            point.x = path_polygon[index].asDouble();
            point.y = path_polygon[index + 1].asDouble();
            point.z = 0;
            new_path.push_back(point);
        }
        if(new_path.size() >= 2)
            paths.push_back(new_path);
        else
        {
            ROS_WARN("Cannot register the freepath selector: the selector \"%s\" should contain more than one point!", path_name.asString().c_str());
        }
    }
    if(paths.size() > 0)
        paths_available = true;
}

CostmapVirtualWalls::CostmapVirtualWalls():
    node("~"),
    subscribe_done(false),
    initialized(false),
    walls_available(false),
    paths_available(false),
    walls(),
    paths(),
    current_x(0),
    current_y(0),
    current_yaw(0)
{
    walls.reserve(16);
    paths.reserve(16);
}

CostmapVirtualWalls::~CostmapVirtualWalls(){}

void CostmapVirtualWalls::onInitialize()
{
    StaticLayer::onInitialize();
    ros::NodeHandle local_node("~/" + name_);
    global_frame_id = layered_costmap_->getGlobalFrameID();
    default_value_ = FREE_SPACE;

    local_node.getParam("enabled", enabled_);
    if(!enabled_)
        return;

    local_node.getParam("walls_topic_name", walls_topic_name);
    walls_topic_name = name_ + walls_topic_name;
    local_node.getParam("global_frame_id", global_frame_id);
    default_cost = local_node.param<int>("default_fill_cost", 0);

    ROS_INFO("Subscribing to Polygon at %s", walls_topic_name.c_str());
    walls_subscriber = node.subscribe(walls_topic_name, 1, &CostmapVirtualWalls::wall_polygon_callback, this);

    std::string walls_json_param_name;
    if(node.searchParam("virtual_walls_json", walls_json_param_name))
    {
        ROS_INFO("Parsing parameter: %s from ROS", walls_json_param_name.c_str());
        node.getParam(walls_json_param_name, walls_json_filename);
        if(boost::filesystem::exists(walls_json_filename))
        {
            ROS_INFO("Parsing file %s", walls_json_filename.c_str());
            parse_walls_json(walls_json_filename);
        }
    }

    current_ = true;
    subscribe_done = true;
    initialized = true;
}

void CostmapVirtualWalls::updateCosts(costmap_2d::Costmap2D &master_grid,
                                      int min_i,
                                      int min_j,
                                      int max_i,
                                      int max_j)
{
    if(!enabled_)
        return;
    StaticLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
    if(walls.empty() && !walls_available && !paths_available)
        return;
    //ROS_INFO("Updating, %lu walls available", walls.size());
    // Retrieving a shared pointer to the costmap data directly into OpenCV
    cv::Mat costmap(master_grid.getSizeInCellsY(), master_grid.getSizeInCellsX(), CV_8UC1, master_grid.getCharMap());
    if(default_cost > 0)
    {
        cv::Mat costmap_mask = costmap.clone();
        cv::inRange(costmap, cv::Scalar(0), cv::Scalar(default_cost), costmap_mask);
        costmap.setTo(cv::Scalar(default_cost), costmap_mask);
    }
    if(walls_available)
    {
        for(auto p = walls.begin(); p != walls.end(); p++)
        {
            geometry_msgs::Polygon polygon = (*p);
            std::vector<cv::Point2i> points;
            volatile bool in_bounds = false;
            for(auto i = polygon.points.begin(); i != polygon.points.end(); i++)
            {
                unsigned int x, y;
                if(master_grid.worldToMap(i->x, i->y, x, y))
                {
                    in_bounds = true;
                    break;
                }
            }
            if(in_bounds)
            {
                for(auto i = polygon.points.begin(); i != polygon.points.end(); i++)
                {
                    int x, y;
                    master_grid.worldToMapEnforceBounds(i->x, i->y, x, y);
                    points.push_back(cv::Point2i(x, y));
                }
                cv::fillConvexPoly(costmap, points, LETHAL_OBSTACLE, cv::LINE_8);
            }
        }
    }
    if(paths_available)
    {
        for(auto p = paths.begin(); p != paths.end(); p++)
        {
            std::vector<geometry_msgs::Point32> path = (*p);
            std::vector<cv::Point2i> points;
            volatile bool in_bounds = false;
            for(auto i = path.begin(); i != path.end(); i++)
            {
                unsigned int x, y;
                if(master_grid.worldToMap(i->x, i->y, x, y))
                {
                    in_bounds = true;
                    break;
                }
            }
            if(in_bounds)
            {
                for(auto i = path.begin(); i != path.end(); i++)
                {
                    int x, y;
                    master_grid.worldToMapEnforceBounds(i->x, i->y, x, y);
                    points.push_back(cv::Point2i(x, y));
                }
                if(points.size() > 1)
                {
                    for(unsigned int i = 0; i < points.size() - 1; i++)
                    {
                        cv::line(costmap, points[i], points[i + 1], FREE_SPACE, 5, cv::LINE_AA);
                    }
                }
            }
        }
    }
    costmap.release();
    //walls_available = false;
    //paths_available = false;
}

