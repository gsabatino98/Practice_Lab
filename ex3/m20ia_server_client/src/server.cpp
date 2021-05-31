#include "ros/ros.h"
#include "m20iA_msgs/m20iA_service.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

bool fk(m20iA_msgs::m20iA_service::Request &req, m20iA_msgs::m20iA_service::Response &res)
{
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotState r_state(kinematic_model);

  moveit::core::robotStateMsgToRobotState(req.robot_state,r_state);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(r_state.getRobotModel()));
  kinematic_state->setToDefaultValues();

  const robot_state::JointModelGroup* joint_model_group = r_state.getRobotModel()->getJointModelGroup("m20iA");
  const std::vector<std::string>& r_link = joint_model_group->getLinkModelNames();

  res.pose_stamped.resize(6);
  res.frame_id.resize(6);
  res.fk_link_name = "flange";
  for (int i = 0; i < 6; i++)
  {
    const Eigen::Isometry3d& link_t = kinematic_state->getGlobalLinkTransform(r_link[i]);

    ROS_INFO_STREAM("link_name:\t"<<r_link[i]);
    
    tf::poseEigenToMsg(link_t, res.pose_stamped[i]);

    res.frame_id[i] = r_link[i];
    
  }

  ROS_INFO("Message sent !!!");
  
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fk_m20ia_server");
  ros::NodeHandle n;

  ROS_INFO("Server started!!!");

  ros::ServiceServer service = n.advertiseService("fk_m20ia", fk);

  ros::spin();

  return 0;
}
