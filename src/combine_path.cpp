#include "combine_path.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(combine_path::CombinePath, nav_core::BaseGlobalPlanner)

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

namespace combine_path{

CombinePath::CombinePath()
  : costmap_ros_(NULL), initialized_(false){}

CombinePath::CombinePath(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
  initialize(name, costmap_ros);
}

void CombinePath::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("step_size", step_size_, costmap_->getResolution());
    private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    private_nh.param("filename", filename_, filename_);
    private_nh.param("default_tolerance", default_tolerance_, 1.5);
    world_model_ = new base_local_planner::CostmapModel(*costmap_); 
    
    if(filename_ != ""){
      ROS_INFO_STREAM("Read waypoints data from " << filename_);
      if(!readFile(filename_)){
        ROS_ERROR("Failed loading waypoints file");
      }
    }else{
      ROS_ERROR("waypoints file doesn't have name");
    }
    initialized_ = true;
  }else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

bool CombinePath::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
  if(!initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  plan.clear();
  
  if(!planned_){
    plan.push_back(start);
    ROS_INFO("wp_size = %d", waypoints_.size());
    navfn::NavfnROS navfnros;
    navfnros.initialize("combine_path", costmap_ros_);
    std::vector<geometry_msgs::PoseStamped> part_path;
    part_path.clear();
    plan_.clear();
    navfnros.makePlan(start, waypoints_[0], plan_);
    for(int i=0; i < waypoints_.size() - 1; i++){
      part_path.clear();
      navfnros.makePlan(waypoints_[i], waypoints_[i+1], part_path);
      for(int j=0; j < part_path.size(); j++){
        plan_.push_back(part_path[j]);
      }
    }
    planned_=true;
    ROS_INFO_STREAM("planned = true");
  }
  plan = plan_;

  return !plan.empty();
}


bool CombinePath::readFile(const std::string &filename){
  waypoints_.clear();
  try{
    std::ifstream ifs(filename.c_str(), std::ifstream::in);
    if(ifs.good() == false){
      return false;
    }
    YAML::Node node;
    
    #ifdef NEW_YAMLCPP
      node = YAML::Load(ifs);
    #else
      YAML::Parser parser(ifs);
      parser.GetNextDocument(node);
    #endif
    
    #ifdef NEW_YAMLCPP
      const YAML::Node &wp_node_tmp = node["waypoints"];
      const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
    #else
      const YAML::Node *wp_node = node.FindValue("waypoints");
    #endif
    
    if(wp_node != NULL){
      for(int i=0; i < wp_node->size(); i++){
        geometry_msgs::PoseStamped pose;
        
        (*wp_node)[i]["point"]["x"] >> pose.pose.position.x;
        (*wp_node)[i]["point"]["y"] >> pose.pose.position.y;
        (*wp_node)[i]["point"]["z"] >> pose.pose.position.z;
        
        waypoints_.push_back(pose);
      }
      for(int i=0; i < wp_node->size(); i++){
        double goal_direction = atan2((waypoints_[i+1].pose.position.y - waypoints_[i].pose.position.y),
                                      (waypoints_[i+1].pose.position.x - waypoints_[i].pose.position.x));
        waypoints_[i].pose.orientation = tf::createQuaternionMsgFromYaw(goal_direction);
      }
    }else{
      return false;
    }
    
    #ifdef NEW_YAMLCPP
      const YAML::Node &fp_node_tmp = node["finish_pose"];
      const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
    #else
      const YAML::Node *fp_node = node.FindValue("finish_pose");
    #endif
    
    if(fp_node != NULL){
      (*fp_node)["pose"]["position"]["x"] >> finish_pose_.pose.position.x;
      (*fp_node)["pose"]["position"]["y"] >> finish_pose_.pose.position.y;
      (*fp_node)["pose"]["position"]["z"] >> finish_pose_.pose.position.z;
      
      (*fp_node)["pose"]["orientation"]["x"] >> finish_pose_.pose.orientation.x;
      (*fp_node)["pose"]["orientation"]["y"] >> finish_pose_.pose.orientation.y;
      (*fp_node)["pose"]["orientation"]["z"] >> finish_pose_.pose.orientation.z;
      (*fp_node)["pose"]["orientation"]["w"] >> finish_pose_.pose.orientation.w;
      
      waypoints_.push_back(finish_pose_);
      for(int i=0; i < waypoints_.size(); i++){
        waypoints_[i].header.frame_id = "/map";
      }
    }else{
      return false;
    }
    
  }catch(YAML::ParserException &e){
      return false;
      
  }catch(YAML::RepresentationException &e){
    return false;
  }

  return true;
}

};//namespace