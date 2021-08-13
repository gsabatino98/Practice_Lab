#include <ros/ros.h>
#include <forward_kinematics_msgs/fk_service.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/RobotState.h>


int main(int argc,char **argv){

    ros::init(argc,argv,"fk_m20ia_client");
    ros::NodeHandle nh;
    ros::ServiceClient client=nh.serviceClient<forward_kinematics_msgs::fk_service>("fk_m20ia");
    forward_kinematics_msgs::fk_service srv;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotState robot_state(kinematic_model);

    moveit_msgs::RobotState robot_msgs_state;

    robot_state::robotStateToRobotStateMsg(robot_state,robot_msgs_state);

    srv.request.robot_state=robot_msgs_state;

    if(client.call(srv)) {
        ROS_INFO_STREAM("Risposta dal server:\n"<<srv.response<<"\n");
    }else{
        ROS_ERROR("Errore nella chiamata!");
        return 1;
    }

    return 0;
}