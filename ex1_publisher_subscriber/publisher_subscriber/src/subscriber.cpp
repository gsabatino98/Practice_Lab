#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <string.h>

using namespace std;  

void chatterCallback(
    const trajectory_msgs::JointTrajectory& jointRobot){
    for (int i = 0; i < 6; i++){
        ROS_INFO_STREAM("I heard from:\t"<<jointRobot.joint_names[i]);
        for (int j = 0; j < 5; j++){
            ROS_INFO_STREAM("Points:\n"<<jointRobot.points[i]);
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber =
    nodeHandle.subscribe("robo_test_sr",10,chatterCallback);
    ros::spin();
    return 0;
}