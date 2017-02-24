#include "combine_path.hpp"

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

namespace combine_path{

CombinePath::CombinePath(ros::NodeHandle& nodeHandle, tf::TransformListener& tf_listener) :
  nodeHandle_(nodeHandle),
  tf_listener_(tf_listener)
{
  readParameters();
  
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_listener_);
  planner_costmap_ros_->start();
  
  if(filename_ != ""){
    ROS_INFO_STREAM("Read waypoints data from " << filename_);
    if(!readFile(filename_)){
      ROS_ERROR("Failed loading waypoints file");
    }
  }else{
    ROS_ERROR("waypoints file doesn't have name");
  }
  
  makePlan("combine_path", planner_costmap_ros_);
  
}

CombinePath::~CombinePath()
{
}

bool CombinePath::readParameters(){
  //nodeHandle_.param("robot_frame", robot_frame_, std::string("/base_link"));
  //nodeHandle_.param("world_frame", world_frame_, std::string("/map"));
  nodeHandle_.param("filename", filename_, filename_);
}

void CombinePath::combinePath(const std::vector<geometry_msgs::PoseStamped> plan){
  for(int i=0; i < plan.size(); i++){
    combined_plan_.push_back(plan[i]);
  }
}

void CombinePath::makePlan(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  navfn::NavfnROS navfnros(name, costmap_ros);
  for(int i=0; i < waypoints_.size(); i++){
    navfnros.makePlan(waypoints_[i], waypoints_[i+1], plan_);
    CombinePath::combinePath(plan_);
  }
  navfnros.publishPlan(combined_plan_, 0.0, 1.0, 0.0, 0.0);
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
                                      (waypoints_[i].pose.position.x - waypoints_[i].pose.position.x));
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
      (*fp_node)["pose"]["position"]["x"] >> finish_pose_.position.x;
      (*fp_node)["pose"]["position"]["y"] >> finish_pose_.position.y;
      (*fp_node)["pose"]["position"]["z"] >> finish_pose_.position.z;
      
      (*fp_node)["pose"]["orientation"]["x"] >> finish_pose_.orientation.x;
      (*fp_node)["pose"]["orientation"]["y"] >> finish_pose_.orientation.y;
      (*fp_node)["pose"]["orientation"]["z"] >> finish_pose_.orientation.z;
      (*fp_node)["pose"]["orientation"]["w"] >> finish_pose_.orientation.w;
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

}//namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "combine_path");
  ros::NodeHandle nh("~");
  tf::TransformListener tf(ros::Duration(10));
  combine_path::CombinePath combinePath(nh, tf);
  ros::spin();
  return 0;
}