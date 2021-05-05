#include <ros/ros.h>
#include <publisher_subscriber_msgs/jointInfo.h>
#include <std_msgs/String.h>
#include <time.h> 
#include <string.h>

using namespace std;  

int main(int argc,char ** argv){
    ros::init(argc,argv,"talker");
    ros::NodeHandle nh;
    ros::Publisher chatterPublisher =
    nh.advertise<publisher_subscriber_msgs::jointInfo> ("robo_custom_msgs",1);
    ros::Rate loopRate(1);

    srand(time(NULL));

    while (ros::ok())
    {
        publisher_subscriber_msgs::jointInfo robotCustomMsgs;

        // Il robot ha 4 giunti
        for (int i = 0; i < 4; i++)
        {
            robotCustomMsgs.joint_names.push_back("joint_"+to_string(i));
        }        

        //Le informazione per ogni nodo corrispondono alla diagnostica in delle sue condizioni
        //ma anche della posizione in 5 tempi diversi
        robotCustomMsgs.diagnostic.resize(5);
        for (int i = 0; i < 5; i++)
        {
            robotCustomMsgs.diagnostic[i].timestamp=rand();
            robotCustomMsgs.diagnostic[i].velocity=rand();
            robotCustomMsgs.diagnostic[i].acceleration=rand();
            robotCustomMsgs.diagnostic[i].satus_sensors=true;
            robotCustomMsgs.diagnostic[i].position.resize(3);
            for (int j = 0; j < 3; j++)
            {
                robotCustomMsgs.diagnostic[i].position[j]=rand();
            }
            
        }
        

        chatterPublisher.publish(robotCustomMsgs);
        

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;    
}