#include "trajectory_planner_plugin.h"
#include "boost/filesystem/operations.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/time.h"

#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <string>

PLUGINLIB_EXPORT_CLASS(dstar_global_planner::DStarGlobalPlanner, nav_core::BaseGlobalPlanner);

namespace dstar_global_planner
{

using namespace libdstar;

void DStarGlobalPlanner::parse_paths_json(const std::string &filename)
{
    if(!boost::filesystem::exists(filename))
    {
        ROS_WARN("File \'%s\' seems not existing, skipping paths collection", filename.c_str());
        return;
    }
    ROS_INFO("Parsing file %s...", filename.c_str());
    std::ifstream paths_file(filename);
    Json::Value json_map;
    json_parser.parse(paths_file, json_map);
    paths_file.close();
    if(json_map.type() != Json::ValueType::objectValue)
    {
        ROS_ERROR("Cannot parse path list from the file: the root JSON object is not a dictionary!");
        return;
    }
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
            ROS_ERROR("Cannot parse path selector %u from the file: the root namespace is not an object!", i);
            return;
        }
        Json::Value path_name = path["name"];
        if(path_name.type() != Json::ValueType::stringValue)
        {
            ROS_ERROR("Cannot parse path selector %u from the file:  the path selector should have a name!", i);
            return;
        }
        ROS_INFO("Parsing freepath selector %s", path_name.asString().c_str());
        Json::Value path_polygon = path["polygon"];
        if(path_polygon.type() != Json::ValueType::arrayValue)
        {
            ROS_ERROR("Cannot parse path selector contents:  the path selector point chain should be defined as an array!");
            return;
        }
        if((path_polygon.size() % 2) != 0)
        {
            ROS_ERROR("Cannot parse object: the point coordinates are not pairable");
            return;
        }
        nav_msgs::Path new_path;
        for(unsigned int index = 0; index < path_polygon.size(); index += 2)
        {
            ROS_INFO("Parsing point %u", index / 2);
            geometry_msgs::PoseStamped point;
            point.pose.position.x = path_polygon[index].asDouble();
            point.pose.position.y = path_polygon[index + 1].asDouble();
            point.pose.position.z = 0;
            new_path.poses.push_back(point);
        }
        if(new_path.poses.size() >= 2)
            ready_paths.push_back(new_path);
        else
            ROS_WARN("Cannot register the freepath selector: the selector \"%s\" should contain more than one point!", path_name.asString().c_str());
    }
}

DStarGlobalPlanner::DStarGlobalPlanner():
    initialized(false),
    ready_paths(),
    json_parser(),
    enable_ready_paths(false)
{
    generator = nullptr;
    state_grid = nullptr;
    initial_path = true;
}

DStarGlobalPlanner::~DStarGlobalPlanner()
{
    if(generator)
        delete generator;
    if(state_grid)
        delete state_grid;
}

DStarGlobalPlanner::DStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
    DStarGlobalPlanner()
{
    initialize(name, costmap_ros);
}

void DStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized)
    {
        ros::NodeHandle node("~" + name);
        node.getParam("goal_distance_threshold", goal_distance_threshold);
        node.getParam("neighbor_distance_threshold", neighbor_distance_threshold);
        node.getParam("occupancy_threshold", occupancy_threshold);
        cutoff_distance = node.param<int>("cutoff_distance", 30);
        node.getParam("trajectory_optimizer/repulsion_gain", repulsion_gain);
        node.getParam("trajectory_optimizer/potential_field_radius", potential_field_radius);
        node.getParam("erosion/enable", erosion);
        node.getParam("erosion/erosion_gap", erosion_gap);
        paths_json_filename = node.param<std::string>("paths/from", "map.json");
        enable_ready_paths = node.param<bool>("paths/enable", false);
        if(enable_ready_paths)
            parse_paths_json(paths_json_filename);
    }
    current_costmap = costmap_ros;
    costmap = current_costmap->getCostmap();
    width = static_cast<long>(costmap->getSizeInCellsX());
    height = static_cast<long>(costmap->getSizeInCellsY());
    ROS_INFO("Obtained map [%li, %li], %f m/pt, filling D* internal costmap...", width, height, costmap->getResolution());
    if(generator)
        delete generator;
    if(state_grid)
        delete state_grid;
    state_grid = new state_map(width, height);
    unsigned char* map_data = costmap->getCharMap();
    //costmap->saveMap("/home/jobot/costmap_test.png");
    for(long i = 0; i < width; i++)
        for(long j = 0; j < height; j++)
        {
            unsigned long pos = (j * width) + i;
            state_grid->point(i, j)->weight_previous = map_data[pos];
            state_grid->point(i, j)->weight = map_data[pos];
            state_grid->point(i, j)->cost_previous = map_data[pos];
            state_grid->point(i, j)->cost_actual = map_data[pos];
            if(map_data[pos] >= occupancy_threshold)
            {
                state_grid->point(i, j)->tag = state_point::stObstacle;
                if(erosion)
                {
                    for(long u = -erosion_gap; u <= erosion_gap; u++)
                        for(long v = erosion_gap; v >= -erosion_gap; v--)
                        {
                            long x_index = i + u;
                            long y_index = j + v;
                            if((x_index >= 0) &&
                                    (x_index < state_grid->get_width()) &&
                                    (y_index >= 0) &&
                                    (y_index < state_grid->get_height()) &&
                                    !((u == 0) && (v == 0)))
                                if(state_grid->point(x_index, y_index)->tag != state_point::stObstacle)
                                    state_grid->point(x_index, y_index)->tag = state_point::stObstacle;
                        }
                }
            }
            else
                state_grid->point(i, j)->tag = state_point::stNew;
        }
    ROS_INFO("D* internal costmap filled, running algorithm");
    generator = new dstar(state_grid);
    generator->set_cutoff_distance(cutoff_distance);
    generator->set_r_field(potential_field_radius);
    generator->set_repulsion_gain(repulsion_gain);
}

bool DStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    plan.clear();
    if(enable_ready_paths && !ready_paths.empty())
    {
        size_t detected_path_index;
        size_t detected_point_index;
        size_t detected_target_index;
        bool found_path = false;
        for(size_t path_index = 0; path_index < ready_paths.size(); path_index++)
        {
            bool found_point = false;
            bool found_target = false;
            for(size_t point_index = 0; point_index < ready_paths[path_index].poses.size(); point_index++)
            {
                double dist = std::sqrt(std::pow(start.pose.position.x - ready_paths[path_index].poses[point_index].pose.position.x, 2) +
                                        std::pow(start.pose.position.y - ready_paths[path_index].poses[point_index].pose.position.y, 2));
                if(dist <= goal_distance_threshold)
                {
                    found_point = true;
                    detected_point_index = point_index;
                    ROS_INFO("Found assigned path for starting point: %zu of the path %zu", detected_point_index, path_index);
                    break;
                }
            }
            if(found_point)
            {
                ROS_INFO("Checking target point...");
                for(size_t point_index = 0; point_index < ready_paths[path_index].poses.size(); point_index++)
                {
                    double dist = std::sqrt(std::pow(goal.pose.position.x - ready_paths[path_index].poses[point_index].pose.position.x, 2) +
                                            std::pow(goal.pose.position.y - ready_paths[path_index].poses[point_index].pose.position.y, 2));
                    if(dist <= goal_distance_threshold)
                    {
                        detected_target_index = point_index;
                        found_target = true;
                        ROS_INFO("Found assigned path for goal point: %zu of the path %zu", detected_target_index, path_index);
                        break;
                    }
                }
                if(found_target && (detected_point_index != detected_target_index))
                {
                    found_path = true;
                    detected_path_index = path_index;
                    break;
                }
            }
        }
        if(found_path)
        {
            ROS_INFO("The existing path between specified points found: %zu", detected_path_index);
            if(detected_target_index < detected_point_index)
            {
                for(size_t i = detected_point_index; i > detected_target_index; i--)
                    plan.push_back(ready_paths[detected_path_index].poses[i]);
                plan.push_back(ready_paths[detected_path_index].poses[detected_target_index]);
            }
            else
            {
                for(size_t i = detected_point_index; i < detected_target_index; i++)
                    plan.push_back(ready_paths[detected_path_index].poses[i]);
                plan.push_back(ready_paths[detected_path_index].poses[detected_target_index]);
            }
            ROS_INFO("Ready-to-use path successfully assigned for current target");
            for(size_t i = 0; i < plan.size(); i++)
            {
                plan[i].header.stamp = ros::Time::now();
                plan[i].header.seq = i;
                plan[i].header.frame_id = current_costmap->getGlobalFrameID();
                if(i != plan.size() - 1)
                {
                    double rel_angle = std::atan2((plan[i + 1].pose.position.y - plan[i].pose.position.y), (plan[i + 1].pose.position.x - plan[i].pose.position.x));
                    tf2::Quaternion q;
                    q.setRPY(0, 0, rel_angle);
                    plan[i].pose.orientation = tf2::toMsg(q);
                }
                else
                    plan[i].pose.orientation = goal.pose.orientation;
            }
            plan.push_back(goal);
            return true;
        }
        else
            ROS_INFO("Existing path not found, running D* global planner");
    }
    tf2::Quaternion current_orientation, destination_orientation;
    int origin_x, origin_y, destination_x, destination_y;
    int c = 0;
    double goal_distance;
    geometry_msgs::PoseStamped pose;
    if(initial_path)
    { // Planned first time
        ROS_INFO("Planning started");
        current_origin = start;
        current_destination = goal;
        goal_distance = std::sqrt(static_cast<double>(std::pow(current_destination.pose.position.x - current_origin.pose.position.x, 2)) +
                                  static_cast<double>(std::pow(current_destination.pose.position.y - current_origin.pose.position.y, 2)));
        if(goal_distance <= goal_distance_threshold)
        {
            ROS_WARN("Attempting to request path to the point located too close (< %f m) from the current position. Dropping down request", goal_distance_threshold);
            return false;
        }
        tf2::fromMsg(start.pose.orientation, current_orientation);
        tf2::fromMsg(goal.pose.orientation, destination_orientation);
        costmap->worldToMapEnforceBounds(current_origin.pose.position.x,
                                         current_origin.pose.position.y,
                                         origin_x,
                                         origin_y);
        costmap->worldToMapEnforceBounds(current_destination.pose.position.x,
                                         current_destination.pose.position.y,
                                         destination_x,
                                         destination_y);
        if(costmap->getCost(origin_x, origin_y) >= occupancy_threshold)
        {
            ROS_WARN("Attempt to generate trajectory from an unknown/occupied area. Dropping down request");
            return false;
        }
        if(costmap->getCost(destination_x, destination_y) >= occupancy_threshold)
        {
            ROS_WARN("Attempt to generate trajectory to an unknown/occupied area. Dropping down request");
            return false;
        }
        ROS_INFO("Generating trajectory (%i, %i) <%u> --> (%i, %i) <%u>", origin_x, origin_y, costmap->getCost(origin_x, origin_y), destination_x, destination_y, costmap->getCost(destination_x, destination_y));
        try
        {
            generator->init_targets(origin_x, origin_y, destination_x, destination_y);
            trajectory.clear();
            trajectory = generator->generate_trajectory();
        }
        catch(dstar_exception)
        {
            ROS_WARN("Empty path generated, probably lost localization");
            return false;
        }

        ROS_INFO("Generated trajectory of %zi waypoints", trajectory.size());
        pose.pose.orientation = tf2::toMsg(current_orientation);
        for(auto i = trajectory.begin(); i != trajectory.end(); i++)
        {
            state_point* p = (*i);
            pose.header.frame_id = current_costmap->getGlobalFrameID();
            pose.header.stamp = ros::Time::now();
            pose.header.seq = ++c;
            costmap->mapToWorld(p->x, p->y, pose.pose.position.x, pose.pose.position.y);
            pose.pose.position.z = 0;
            auto next_point_iterator = std::next(i);
            if(next_point_iterator != trajectory.end())
            {
                state_point* p2 = (*next_point_iterator);
                double next_x, next_y;
                costmap->mapToWorld(p2->x, p2->y, next_x, next_y);
                double closest_distance = std::sqrt(std::pow(next_y - pose.pose.position.y, 2) + std::pow(next_x - pose.pose.position.x, 2));
                double rel_angle = std::atan2((next_y - pose.pose.position.y), (next_x - pose.pose.position.x));
                tf2::Quaternion q;
                q.setRPY(0, 0, rel_angle);
                pose.pose.orientation = tf2::toMsg(q);
                if(closest_distance < goal_distance_threshold)
                    continue;
            }
            else
            {
                pose.pose.orientation = tf2::toMsg(destination_orientation);
            }
            plan.push_back(pose);
        }
        plan.push_back(goal);
        initial_path = false;
    }
    else
    { // Replanning
        ROS_INFO("Replanning started");
        double goal_distance = std::sqrt(static_cast<double>(std::pow(goal.pose.position.x - current_destination.pose.position.x, 2)) +
                                         static_cast<double>(std::pow(goal.pose.position.y - current_destination.pose.position.y, 2)));
        if(goal_distance > neighbor_distance_threshold)
        {
            ROS_WARN("Attempting to replan the path to not the same destination point: dropping down the previous");
            initial_path = true;
            return false;
        }
        current_origin = start;
        tf2::fromMsg(start.pose.orientation, current_orientation);
        costmap->worldToMapEnforceBounds(current_origin.pose.position.x,
                                         current_origin.pose.position.y,
                                         origin_x,
                                         origin_y);
        costmap->worldToMapEnforceBounds(current_destination.pose.position.x,
                                         current_destination.pose.position.y,
                                         destination_x,
                                         destination_y);
        ROS_INFO("Replanning trajectory (%i, %i) <%u> --> (%i, %i) <%u>", origin_x, origin_y, costmap->getCost(origin_x, origin_y), destination_x, destination_y, costmap->getCost(destination_x, destination_y));
        try
        {
            trajectory.clear();
            trajectory = generator->replan_trajectory(origin_x, origin_y, state_grid->point(origin_x, origin_y)->cost_actual);
        }
        catch(dstar_exception)
        {
            ROS_WARN("Empty path generated, probably wrong localization");
            return false;
        }
        for(auto i = trajectory.begin(); i != trajectory.end(); i++)
        {
            state_point* p = (*i);
            pose.header.frame_id = current_costmap->getGlobalFrameID();
            pose.header.stamp = ros::Time::now();
            pose.header.seq = ++c;
            costmap->mapToWorld(p->x, p->y, pose.pose.position.x, pose.pose.position.y);
            pose.pose.position.z = 0;
            auto next_point_iterator = std::next(i);
            if(next_point_iterator != trajectory.end())
            {
                state_point* p2 = (*next_point_iterator);
                double next_x, next_y;
                costmap->mapToWorld(p2->x, p2->y, next_x, next_y);
                double closest_distance = std::sqrt(std::pow(next_y - pose.pose.position.y, 2) + std::pow(next_x - pose.pose.position.x, 2));
                double rel_angle = std::atan2((next_y - pose.pose.position.y), (next_x - pose.pose.position.x));
                tf2::Quaternion q;
                q.setRPY(0, 0, rel_angle);
                pose.pose.orientation = tf2::toMsg(q);
                if(closest_distance < goal_distance_threshold)
                    continue;
            }
            else
            {
                pose.pose.orientation = tf2::toMsg(destination_orientation);
            }
            plan.push_back(pose);
        }
        plan.push_back(goal);
    }
    return true;
}

bool DStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan, double &cost)
{
    return true;
}

}
