#include<ros/ros.h>
#include<tf/tf.h>
#include<tf2/utils.h>
#include<tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>

void print_rotMat_eulerRPY_axisAngle(double x, double y, double z, double w){
    tf2::Quaternion q(x,y,z,w);
    
    tf2::Matrix3x3 rotMat(q);

    double printMat [3][3];
    for (int i = 0; i < 3; i++)
    {
        printMat[0][i]=rotMat.getRow(0)[i];
        printMat[1][i]=rotMat.getRow(1)[i];
        printMat[2][i]=rotMat.getRow(2)[i];
    }
    
    ROS_INFO("Rotation Matrix:");
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO_STREAM("\t["<<printMat[i][0]<<", "<<printMat[i][1]<<", "<<printMat[i][2]<<"]");
    }
    

    //PRINT EULER ROLL PITCH YAW
    double roll,pitch,yaw;
    rotMat.getRPY(roll,pitch,yaw);
    ROS_INFO_STREAM("\nEuler angles:\nroll:\t"<<roll<<"\npitch:\t"<<pitch<<"\nyaw:\t"<< yaw);

    //PRINT AXIS ANGLE
    tf2::Vector3 axis = q.getAxis();
    double tetha = q.getAngle();

    ROS_INFO_STREAM("\nAxis-Angle:\nr:\t[" << axis[0] << " " << axis[1] << " " << axis[2] << "], tetha: " << tetha<<"\n");

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf2_listener");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);

    std::vector<geometry_msgs::TransformStamped> transformStamped;
    
    while(ros::ok()) {
        
        try{
            
            transformStamped.push_back(tfBuffer.lookupTransform("base_link", "flange", ros::Time(0)));
        }
        catch (tf2::TransformException &exception) {
        ROS_WARN("%s", exception.what());
        ros::Duration(1.0).sleep();
        continue;
        }

        for (int i = 0; i < 6; i++)
        {
            try{
            transformStamped.push_back(tfBuffer.lookupTransform("link"+std::to_string(i+1),"flange", ros::Time(0)));
            }
            catch (tf2::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            ros::Duration(1.0).sleep();
            continue;
            }
        }
        for (int i = 0; i < 6; i++)
        {
            double x = transformStamped[i].transform.rotation.x;
            double y = transformStamped[i].transform.rotation.y;
            double z = transformStamped[i].transform.rotation.z;
            double w = transformStamped[i].transform.rotation.w;
            geometry_msgs::Vector3 t=transformStamped[i].transform.translation;

            ROS_INFO_STREAM("\n\nChild frame id:\t"<<transformStamped[i].child_frame_id<<"\nFrame id:\t"<<transformStamped[i].header.frame_id<<"\n"<<
            "\nQuaternion:\t["<<x<<", "<<y<<", "<<z<<", "<<w<<"]\n"<<
            "\nTranslation:\t["<<t.x<<", "<<t.y<<", "<<t.z<<"]\n\n");
            print_rotMat_eulerRPY_axisAngle(x,y,z,w);
        }
        
        rate.sleep();
    }
    return 0;
};