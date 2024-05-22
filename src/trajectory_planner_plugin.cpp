#include "trajectory_planner_plugin.h"
#include "boost/filesystem/operations.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/path.hpp>

#include <cmath>
#include <string>

// #include <memory>
// #include "nav2_util/node_utils.hpp"

namespace dstar_global_planner
{

using namespace libdstar;

void DStarGlobalPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
){
    node_ = parent.lock();
    RCLCPP_INFO(node_->get_logger(), "Configurating plugin %s of type NavfnPlanner", name_.c_str());
    tf_ = tf;
    name_ = name;
    global_frame_ = costmap_ros->getGlobalFrameID();
    //-----added here for now
    initialized = false;
    generator = nullptr;
    state_grid = nullptr;
    initial_path = true;
    //-----added here for now
    if(!initialized){

      nav2_util::declare_parameter_if_not_declared(node_, name_ + "." +"goal_distance_threshold", rclcpp::PARAMETER_DOUBLE); 
      node_->get_parameter(name_ + "." +"goal_distance_threshold", goal_distance_threshold);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "neighbor_distance_threshold", rclcpp::PARAMETER_DOUBLE);
      node_->get_parameter(name_ + "." + "neighbor_distance_threshold", neighbor_distance_threshold);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "occupancy_threshold", rclcpp::PARAMETER_INTEGER);
      node_->get_parameter(name_ + "." + "occupancy_threshold", occupancy_threshold);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "cutoff_distance", rclcpp::ParameterValue(30));
      node_->get_parameter(name_ + "." + "cutoff_distance", cutoff_distance);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "trajectory_optimizer.repulsion_gain", rclcpp::PARAMETER_DOUBLE);
      node_->get_parameter(name_ + "." + "trajectory_optimizer.repulsion_gain", repulsion_gain);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "trajectory_optimizer.potential_field_radius", rclcpp::PARAMETER_DOUBLE);
      node_->get_parameter(name_ + "." + "trajectory_optimizer.potential_field_radius", potential_field_radius);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "erosion.enable", rclcpp::PARAMETER_BOOL);
      node_->get_parameter(name_ + "." + "erosion.enable", erosion);
      nav2_util::declare_parameter_if_not_declared(node_,name_ + "." + "erosion.erosion_gap", rclcpp::PARAMETER_INTEGER);
      node_->get_parameter(name_ + "." + "erosion.erosion_gap", erosion_gap);
      
      nav2_util::declare_parameter_if_not_declared(node_, "paths.from", rclcpp::ParameterValue("map.json"));
      paths_json_filename = node_->get_parameter("paths.from").as_string();
      nav2_util::declare_parameter_if_not_declared(node_, "paths.enable", rclcpp::ParameterValue(false));
      enable_ready_paths = node_->get_parameter("paths.enable").as_bool();
      if(enable_ready_paths)
        RCLCPP_WARN(node_->get_logger(),"NOT IMPLEMENTED");
        // parse_paths_json(paths_json_filename);
    }
    current_costmap = costmap_ros;
    RCLCPP_WARN(node_->get_logger(),"getting costmap");
    costmap = costmap_ros->getCostmap();
    width = static_cast<long>(costmap->getSizeInCellsX());
    height = static_cast<long>(costmap->getSizeInCellsY());
    RCLCPP_INFO(node_->get_logger(),"Obtained map [%li, %li], %f m/pt, filling D* internal costmap...", width, height, costmap->getResolution());
    if(generator)
        delete generator;
    if(state_grid)
        delete state_grid;
    state_grid = new state_map(width, height);
    unsigned char* map_data = costmap->getCharMap();
    // costmap->saveMap("/home/jobot/costmap_test.png");
    for(long i = 0; i < width; i++)
      for(long j = 0; j < height; j++)
      {
        unsigned long pos = (j * width) + i;
        state_grid->point(i, j)->weight_previous = map_data[pos];
        state_grid->point(i, j)->weight = map_data[pos];
        state_grid->point(i, j)->cost_previous = map_data[pos];
        state_grid->point(i, j)->cost_actual = map_data[pos];
        if(map_data[pos] >= occupancy_threshold){
          state_grid->point(i, j)->tag = state_point::stObstacle;
          if(erosion){
            for(long u = -erosion_gap; u <= erosion_gap; u++)
              for(long v = erosion_gap; v >= -erosion_gap; v--){
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
    RCLCPP_INFO(node_->get_logger(),"D* internal costmap filled, running algorithm");
    generator = new dstar(state_grid);
    generator->set_cutoff_distance(cutoff_distance);
    generator->set_r_field(potential_field_radius);
    generator->set_repulsion_gain(repulsion_gain);
    
    //tmp fix
    // delete state_grid;
    // delete generator;

}
void DStarGlobalPlanner::activate(){
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type NavfnPlanner", name_.c_str());
  // generator = nullptr;
  // state_grid = nullptr;
  // initial_path = true;

}

void DStarGlobalPlanner::cleanup(){
  if(generator)
    delete generator;
  if(state_grid)
    delete state_grid;
  RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner", name_.c_str());
}

void DStarGlobalPlanner::deactivate(){
    RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",name_.c_str());
}

nav_msgs::msg::Path DStarGlobalPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
){
    nav_msgs::msg::Path global_path;

//////////////////////////////////////////////////////


  // std::vector<geometry_msgs::msg::PoseStamped> plan;
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  
  global_path.poses.clear();

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
          RCLCPP_INFO(node_->get_logger(),"Found assigned path for starting point: %zu of the path %zu", detected_point_index, path_index);
          break;
        }
      }
      if(found_point)
      {
        RCLCPP_INFO(node_->get_logger(),"Checking target point...");
        for(size_t point_index = 0; point_index < ready_paths[path_index].poses.size(); point_index++)
        {
          double dist = std::sqrt(std::pow(goal.pose.position.x - ready_paths[path_index].poses[point_index].pose.position.x, 2) +
                                  std::pow(goal.pose.position.y - ready_paths[path_index].poses[point_index].pose.position.y, 2));
          if(dist <= goal_distance_threshold)
          {
            detected_target_index = point_index;
            found_target = true;
            RCLCPP_INFO(node_->get_logger(),"Found assigned path for goal point: %zu of the path %zu", detected_target_index, path_index);
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
      RCLCPP_INFO(node_->get_logger(),"The existing path between specified points found: %zu", detected_path_index);
      if(detected_target_index < detected_point_index)
      {
        for(size_t i = detected_point_index; i > detected_target_index; i--)
          global_path.poses.push_back(ready_paths[detected_path_index].poses[i]);
        global_path.poses.push_back(ready_paths[detected_path_index].poses[detected_target_index]);
      }
      else
      {
        for(size_t i = detected_point_index; i < detected_target_index; i++)
          global_path.poses.push_back(ready_paths[detected_path_index].poses[i]);
        global_path.poses.push_back(ready_paths[detected_path_index].poses[detected_target_index]);
      }
      RCLCPP_INFO(node_->get_logger(),"Ready-to-use path successfully assigned for current target");
      for(size_t i = 0; i < global_path.poses.size(); i++)
      {
        global_path.poses[i].header.stamp = rclcpp::Clock{}.now();
        // no sequence in ros2 in header
        // global_path[i].header.seq = i;
        global_path.poses[i].header.frame_id = current_costmap->getGlobalFrameID();
        if(i != global_path.poses.size() - 1)
        {
          double rel_angle = std::atan2((global_path.poses[i + 1].pose.position.y - global_path.poses[i].pose.position.y), (global_path.poses[i + 1].pose.position.x - global_path.poses[i].pose.position.x));
          tf2::Quaternion q;
          q.setRPY(0, 0, rel_angle);
          global_path.poses[i].pose.orientation = tf2::toMsg(q);
        }
        else
          global_path.poses[i].pose.orientation = goal.pose.orientation;
      }
      //global_path.poses.push_back(goal);
      return global_path;
    }
    else
        RCLCPP_INFO(node_->get_logger(),"Existing path not found, running D* global planner");
  }
  tf2::Quaternion current_orientation, destination_orientation;
  int origin_x, origin_y, destination_x, destination_y;
  int c = 0;
  double goal_distance;
  geometry_msgs::msg::PoseStamped pose;
  if(initial_path)
  { // Planned first time
    RCLCPP_INFO(node_->get_logger(),"Planning started");
    current_origin = start;
    current_destination = goal;
    goal_distance = std::sqrt(static_cast<double>(std::pow(current_destination.pose.position.x - current_origin.pose.position.x, 2)) +
                              static_cast<double>(std::pow(current_destination.pose.position.y - current_origin.pose.position.y, 2)));
    if(goal_distance <= goal_distance_threshold)
    {
      RCLCPP_WARN(node_->get_logger(),"Attempting to request path to the point located too close (< %f m) from the current position. Dropping down request", goal_distance_threshold);
      global_path.poses.clear();
      return global_path;
      // return false;
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
    RCLCPP_WARN(node_->get_logger(),"Cost of start position (%d, %d) is %d on (%d)", origin_x, origin_y, costmap->getCost(origin_x, origin_y), occupancy_threshold);
    if(costmap->getCost(origin_x, origin_y) >= occupancy_threshold)
    {
      RCLCPP_WARN(node_->get_logger(),"Cost of start position (%d, %d) is %d on (%d)", origin_x, origin_y, costmap->getCost(origin_x, origin_y), occupancy_threshold);
      RCLCPP_WARN(node_->get_logger(),"Attempt to generate trajectory from an unknown/occupied area. Dropping down request");
      global_path.poses.clear();
      return global_path;
      // return false;
    }
    RCLCPP_WARN(node_->get_logger(),"Cost of final position (%d, %d) is %d on (%d)", destination_x, destination_y, costmap->getCost(destination_x, destination_y), occupancy_threshold);
    if(costmap->getCost(destination_x, destination_y) >= occupancy_threshold)
    {
      RCLCPP_WARN(node_->get_logger(),"Attempt to generate trajectory to an unknown/occupied area. Dropping down request");
      global_path.poses.clear();
      return global_path;
      // return false;
    }
    RCLCPP_INFO(node_->get_logger(),"Generating trajectory (%i, %i) <%u> --> (%i, %i) <%u>", origin_x, origin_y, costmap->getCost(origin_x, origin_y), destination_x, destination_y, costmap->getCost(destination_x, destination_y));
    try
    {
      generator->init_targets(origin_x, origin_y, destination_x, destination_y);
      trajectory.clear();
      trajectory = generator->generate_trajectory();
    }
    catch(dstar_exception)
    {
      RCLCPP_WARN(node_->get_logger(),"Empty path generated, probably lost localization");
      global_path.poses.clear();
      return global_path;
      // return false;
    }

    RCLCPP_INFO(node_->get_logger(),"Generated trajectory of %zi waypoints", trajectory.size());
    pose.pose.orientation = tf2::toMsg(current_orientation);
    for(auto i = trajectory.begin(); i != trajectory.end(); i++)
    {
      state_point* p = (*i);
      //pose.header.frame_id = current_costmap->getGlobalFrameID();
      // pose.header.stamp = rclcpp::Clock{}.now();
      // no sequence in ros2 in header
      // pose.header.seq = i;
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
        global_path.poses.push_back(pose);
    }
    //global_path.poses.push_back(goal);
    initial_path = false;
  }
  else
  { // Replanning
    RCLCPP_INFO(node_->get_logger(),"Replanning started");
    double goal_distance = std::sqrt(static_cast<double>(std::pow(goal.pose.position.x - current_destination.pose.position.x, 2)) +
                                      static_cast<double>(std::pow(goal.pose.position.y - current_destination.pose.position.y, 2)));
    if(goal_distance > neighbor_distance_threshold)
    {
      RCLCPP_WARN(node_->get_logger(),"Attempting to replan the path to not the same destination point: dropping down the previous");
      initial_path = true;
      global_path.poses.clear();
      return global_path;
      // return false;
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
    RCLCPP_INFO(node_->get_logger(),"Replanning trajectory (%i, %i) <%u> --> (%i, %i) <%u>", origin_x, origin_y, costmap->getCost(origin_x, origin_y), destination_x, destination_y, costmap->getCost(destination_x, destination_y));
    try
    {
      trajectory.clear();
      trajectory = generator->replan_trajectory(origin_x, origin_y, state_grid->point(origin_x, origin_y)->cost_actual);
    }
    catch(dstar_exception)
    {
      RCLCPP_WARN(node_->get_logger(),"Empty path generated, probably wrong localization");
      global_path.poses.clear();
      return global_path;
      // return false;
    }
    for(auto i = trajectory.begin(); i != trajectory.end(); i++)
    {
      state_point* p = (*i);
      //pose.header.frame_id = current_costmap->getGlobalFrameID();
      pose.header.stamp = rclcpp::Clock{}.now();
      // no sequence in ros2 in header
      // pose.header.seq = i;
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
      global_path.poses.push_back(pose);
    }
    //global_path.poses.push_back(goal);
  }

return global_path;

}


}

// https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html
#include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(dstar_global_planner::DStarGlobalPlanner, nav2_core::BaseGlobalPlanner);
PLUGINLIB_EXPORT_CLASS(dstar_global_planner::DStarGlobalPlanner, nav2_core::GlobalPlanner);
