#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <fstream>
#include <string>
#include <math.h>
#include <limits>

namespace combine_path{

class CombinePath
{
public:
  CombinePath(ros::NodeHandle& nh, tf::TransformListener& tf_lisner);
  virtual ~CombinePath();
  bool readParameters();
  bool readFile(const std::string &filename);
  void combinePath(const std::vector<geometry_msgs::PoseStamped> plan);
  void makePlan(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

private:
  ros::NodeHandle& nodeHandle_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  std::vector<geometry_msgs::PoseStamped> combined_plan_;
  geometry_msgs::Pose finish_pose_;
  tf::TransformListener& tf_listener_;
  costmap_2d::Costmap2DROS* planner_costmap_ros_;
  std::string robot_frame_, world_frame_;
  std::string filename_ = "";
};
}//namespace