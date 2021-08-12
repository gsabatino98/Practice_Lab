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

        // Il robot ha 4 giunti
        for (int i = 0; i < 4; i++)
        {
            robotMsgs.joint_name.push_back("joint_"+to_string(i));
        }

        //Le informazione per ogni nodo corrispondono alla diagnostica in delle sue condizioni
        //ma anche della posizione in 5 tempi diversi
        robotMsgs.diagnostic.resize(5);
        for (int i = 0; i < 5; i++)
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