#include <ros/ros.h>
#include <publisher_subscriber_msgs/diagnosticRobot.h>
#include <publisher_subscriber_msgs/jointInfo.h>
#include <std_msgs/String.h>
#include <string.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;  

void chatterCallback(
    const publisher_subscriber_msgs::jointInfo& robotCustomMsgs){
    for (int i = 0; i < 4; i++){
        ROS_INFO_STREAM("I heard from:\t"<<robotCustomMsgs.joint_names[i]);
        
        ROS_INFO_STREAM("Diagnosis occurred:\t"<<robotCustomMsgs.diagnostic[i].timestamp);
        ROS_INFO_STREAM("Joint position:\tx:\t"<<robotCustomMsgs.diagnostic[i].position[0]);
        ROS_INFO_STREAM("               \ty:\t"<<robotCustomMsgs.diagnostic[i].position[1]);
        ROS_INFO_STREAM("               \tz:\t"<<robotCustomMsgs.diagnostic[i].position[2]);
        ROS_INFO_STREAM("velocity:\t"<<robotCustomMsgs.diagnostic[i].velocity);
        ROS_INFO_STREAM("acceleration:\t"<<robotCustomMsgs.diagnostic[i].acceleration);
        if(robotCustomMsgs.diagnostic[i].satus_sensors==true){
            ROS_INFO_STREAM("working satus:\tTRUE\n\n");  
        }else{
            ROS_INFO_STREAM("working satus:\tFALSE\n\n");
        }
                  
        
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener_custom_msgs");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber =
    nodeHandle.subscribe("robo_custom_msgs",10,chatterCallback);
    ros::spin();
    return 0;
}