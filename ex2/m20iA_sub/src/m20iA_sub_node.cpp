#include<ros/ros.h>
#include<tf/tf.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>

void print_rotMat_eulerRPY_axisAngle(double x, double y, double z, double w){
    tf::Quaternion q(x,y,z,w);
    
    tf::Matrix3x3 rotMat(q);

    double printMat [3][3];
    for (int i = 0; i < 3; i++)
    {
        printMat[0][i]=rotMat.getRow(0)[i];
        printMat[1][i]=rotMat.getRow(1)[i];
        printMat[2][i]=rotMat.getRow(2)[i];
    }
    
    ROS_INFO("\n\nRotation Matrix:\n");
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO_STREAM("\t["<<printMat[i][0]<<", "<<printMat[i][1]<<", "<<printMat[i][2]<<"]");
    }
    

    //PRINT EULER ROLL PITCH YAW
    double roll,pitch,yaw;
    rotMat.getRPY(roll,pitch,yaw);
    ROS_INFO_STREAM("\n\nEuler angles:\nroll:\t"<<roll<<"\npitch:\t"<<pitch<<"\nyaw:\t"<< yaw);

    //PRINT AXIS ANGLE
    tf::Vector3 axis = q.getAxis();
    double tetha = q.getAngle();

    ROS_INFO_STREAM("\n\nAxis-Angle:\nr:\t[" << axis[0] << " " << axis[1] << " " << axis[2] << "], tetha: " << tetha<<"\n");

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf2_listener");
    tf2_ros::Buffer tfBuffer[7];
    tf2_ros::TransformListener tfListener1(tfBuffer[0]),tfListener2(tfBuffer[1]),tfListener3(tfBuffer[2]),tfListener4(tfBuffer[3]),tfListener5(tfBuffer[4]),
    tfListener6(tfBuffer[5]),tfListener7(tfBuffer[6]);

    ros::Rate rate(10.0);
    
    while(ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try{

            transformStamped = tfBuffer[0].lookupTransform("base_link", "flange", ros::Time(0));

                double x = transformStamped.transform.rotation.x;
                double y = transformStamped.transform.rotation.y;
                double z = transformStamped.transform.rotation.z;
                double w = transformStamped.transform.rotation.w;
                geometry_msgs::Vector3 t=transformStamped.transform.translation;

                ROS_INFO_STREAM("\n\nChild frame id:\t"<<transformStamped.child_frame_id<<"\nFrame id:\t"<<transformStamped.header.frame_id<<"\n");
                ROS_INFO_STREAM("Quaternion:\t["<<x<<", "<<y<<", "<<z<<", "<<w<<"]\n");
                ROS_INFO_STREAM("Translation:\t["<<t.x<<", "<<t.y<<", "<<t.z<<"]\n");
                print_rotMat_eulerRPY_axisAngle(x,y,z,w);

            for (int i = 0; i < 6; i++)
            {
                transformStamped = tfBuffer[i].lookupTransform("link"+std::to_string(i+1),"flange", ros::Time(0));

                double x = transformStamped.transform.rotation.x;
                double y = transformStamped.transform.rotation.y;
                double z = transformStamped.transform.rotation.z;
                double w = transformStamped.transform.rotation.w;
                geometry_msgs::Vector3 t=transformStamped.transform.translation;

                ROS_INFO_STREAM("\n\nChild frame id:\t"<<transformStamped.child_frame_id<<"\nFrame id:\t"<<transformStamped.header.frame_id<<"\n");
                ROS_INFO_STREAM("Quaternion:\t["<<x<<", "<<y<<", "<<z<<", "<<w<<"]\n");
                ROS_INFO_STREAM("Translation:\t["<<t.x<<", "<<t.y<<", "<<t.z<<"]\n");
                print_rotMat_eulerRPY_axisAngle(x,y,z,w);
            }
        }
        catch (tf2::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    return 0;
};