#ifndef COMBINE_PATH_H
#define COMBINE_PATH_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <navfn/navfn.h>
#include <nav_core/base_global_planner.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <fstream>
#include <string>
#include <math.h>
#include <limits>



namespace combine_path{

class CombinePath : public nav_core::BaseGlobalPlanner
{
public:
  CombinePath();
  CombinePath(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool readFile(const std::string &filename);
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  std::vector<geometry_msgs::PoseStamped> combined_path_;
private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  geometry_msgs::PoseStamped finish_pose_;
  std::string robot_frame_, world_frame_;
  base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
  std::string filename_ = "/home/tera/catkin_ws/src/orne_navigation/orne_navigation_executor/waypoints_cfg/waypoints_alpha.yaml";
  bool initialized_;
  bool planned_ = false;
  bool first_wp_ = false;
  double step_size_, min_dist_from_robot_, default_tolerance_;
  std::vector<geometry_msgs::PoseStamped> part_path_;
  
};
};//namespace
#endif