#include <ros/ros.h>
#include <forward_kinematics_msgs/fk_service.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
// #include <kdl_kinematics_plugin/kdl_kinematics_plugin.h>

bool fk(forward_kinematics_msgs::fk_service::Request &req, forward_kinematics_msgs::fk_service::Response &res)
{
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState r_state(kinematic_model);
    
    moveit::core::robotStateMsgToRobotState(req.robot_state,r_state);

    const robot_state::JointModelGroup* joint_model_group = r_state.getRobotModel()->getJointModelGroup("fanuc_m20ia");
    const std::vector<std::string>& r_link = joint_model_group->getLinkModelNames();

    int linkNumber = r_link.size()-1;
    ROS_INFO_STREAM("\n\n"<<linkNumber);
    res.pose_stamped.resize(linkNumber);
    res.frame_id.resize(linkNumber);
    res.fk_link_name = r_link[linkNumber];   

    for (int i = 0; i < linkNumber; i++)
    {
        const Eigen::Isometry3d& link_t = r_state.getGlobalLinkTransform(r_link[i]);

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
    ros::NodeHandle nh;

    ROS_INFO("Server started !!!!!");

    ros::ServiceServer service = nh.advertiseService("fk_m20ia", fk);

    ros::spin();

    return 0;
}