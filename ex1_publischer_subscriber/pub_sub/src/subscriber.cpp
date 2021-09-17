#include <ros/ros.h>
#include <pub_sub_msgs/diagnosticInfo.h>
#include <pub_sub_msgs/jointsInfo.h>
#include <std_msgs/String.h>
#include <string.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;  

void chatterCallback(
    const pub_sub_msgs::jointsInfo& robotMsgs){
    for (int i = 0; i < 6; i++){
        ROS_INFO_STREAM("I heard from:\t"<<robotMsgs.joint_name[i]<<
        "\nDiagnosis occurred:\t"<<robotMsgs.diagnostic[i].timestamp<<
        "\nJoint position:\t"<<robotMsgs.diagnostic[i].position<<
        "\nvelocity:\t"<<robotMsgs.diagnostic[i].velocity<<
        "\nacceleration:\t"<<robotMsgs.diagnostic[i].acceleration);
        if(robotMsgs.diagnostic[i].satus_sensors==true){
            ROS_INFO_STREAM("working satus:\tTRUE\n\n");  
        }else{
            ROS_INFO_STREAM("working satus:\tFALSE\n\n");
        }
                  
        
    }
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("joints_position",10,chatterCallback);
    ros::spin();
    return 0;
}