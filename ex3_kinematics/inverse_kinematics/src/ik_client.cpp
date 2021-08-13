#include <ros/ros.h>
#include <inverse_kinematics_msgs/ik_actionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

void doneIK_CB(const actionlib::SimpleClientGoalState& state,const inverse_kinematics_msgs::ik_actionResultConstPtr& result){
    ROS_INFO_STREAM("Azione finita con stato:\n\t\t" << state.getText().c_str());
    if (result->robot_state.size()!=0)
    {
        for (int i = 0; i < result->robot_state.size(); i++)
        {
            ROS_INFO_STREAM("\n"<<result->robot_state[i]<<"\n");
        }
    }
    ros::shutdown();
    }

void activeServer_CB()
    {
    ROS_INFO("Server avviato, elaborazione GOAL !!!!!");
    }

void feedbackServer_CB(const inverse_kinematics_msgs::ik_actionFeedbackConstPtr& feedback)
    {
    ROS_INFO_STREAM("Una nuova posa e' stata trovata !!!!! \n");
    ROS_INFO_STREAM("\n"<<feedback->robot_state<<"\n");
    }

int main(int argc,char **argv){

    ros::init(argc,argv,"ik_fanuc_m20ia_client");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<inverse_kinematics_msgs::ik_actionAction> ac("ik_fanuc_m20ia",true);
    inverse_kinematics_msgs::ik_actionGoal diesired_goal;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(kinematic_model);

    Eigen::Isometry3d pose_goal = robot_state.getGlobalLinkTransform("flange");
    tf::poseEigenToMsg(pose_goal, diesired_goal.pose_goal);
    ROS_INFO("Posizioni prima della random:");
    robot_state.printStatePositionsWithJointLimits(robot_state.getJointModelGroup("fanuc_m20ia"));
    robot_state.setToRandomPositions();
    ROS_INFO("Posizioni dopo la random:");
    robot_state.printStatePositionsWithJointLimits(robot_state.getJointModelGroup("fanuc_m20ia"));

    robot_state::robotStateToRobotStateMsg(robot_state,diesired_goal.robot_state);
    
    ac.waitForServer();
    
    ROS_INFO_STREAM("Goal inviato:\n"<<diesired_goal);
    
    ac.sendGoal(diesired_goal, &doneIK_CB, &activeServer_CB, &feedbackServer_CB);
    ac.waitForResult();
    return 0;
}