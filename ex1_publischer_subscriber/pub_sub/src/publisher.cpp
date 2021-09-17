#include <ros/ros.h>
#include <pub_sub_msgs/jointsInfo.h>
#include <std_msgs/String.h>
#include <string.h>

using namespace std;  

int main(int argc,char ** argv){

    // istanzio il nodo
    ros::init(argc,argv,"talker");
    // istanzio una interfaccia per gestire il nodo
    ros::NodeHandle nh;
    // instanzio un oggetto di tipo publisher per pubbilicare le posizioni dei giunti su un topic ("joints_position")
    ros::Publisher publisher = nh.advertise<pub_sub_msgs::jointsInfo>("joints_position",1);
    ros::Rate loopRate(1);

    // uso un seed per il random generator
    srand(time(NULL));

        while (ros::ok())
    {
        pub_sub_msgs::jointsInfo robotMsgs;

        for (int i = 0; i < 6; i++)
        {
            robotMsgs.joint_name.push_back("joint_"+to_string(i));
        }

        robotMsgs.diagnostic.resize(6);
        for (int i = 0; i < 6; i++)
        {
            robotMsgs.diagnostic[i].timestamp=rand();
            robotMsgs.diagnostic[i].velocity=rand();
            robotMsgs.diagnostic[i].acceleration=rand();
            robotMsgs.diagnostic[i].satus_sensors=true;
            robotMsgs.diagnostic[i].position=rand();
            
        }
        

        publisher.publish(robotMsgs);     

        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;  
}