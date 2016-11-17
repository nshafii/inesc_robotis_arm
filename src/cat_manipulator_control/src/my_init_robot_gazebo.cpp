#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen_conversions/eigen_msg.h>

#include <pthread.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

# define MAX_JOINT_ID 6

std::string joint_name [ MAX_JOINT_ID + 1 ] =
{ "None", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

ros::Publisher joint_pub [ MAX_JOINT_ID + 1 ];

std::vector<double> joint_values;

void joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{

    for ( int _joint_index = 0; _joint_index < msg->name.size(); _joint_index++ )
    {
        double _joint_position = msg->position[ _joint_index ];

       joint_values[_joint_index] = _joint_position;
    }
}

bool hasRobotMoved(const sensor_msgs::JointState& js){

    ros::spinOnce();
    ros::Duration(5.0).sleep();
    ros::spinOnce();

    std::cout << " JOINT VALUES : " << std::endl;
    for (int j=0; j< 6; j++)
        std::cout << j << " : " << joint_values[j] << std::endl;

    std::cout << js << std::endl;

    double error_allowed = 0.1;
    if ( fabs(joint_values[0] - js.position[0]) < error_allowed &
         fabs(joint_values[1] - js.position[1]) < error_allowed &
         fabs(joint_values[2] - js.position[2]) < error_allowed &
         fabs(joint_values[3] - js.position[3]) < error_allowed &
         fabs(joint_values[4] - js.position[4]) < error_allowed &
         fabs(joint_values[5] - js.position[5]) < error_allowed)
    {
        ROS_INFO("CHECKUP : Robot DID Move");
        return true;
    }
    else
    {
       ROS_INFO("CHECKUP : Robot DID NOT Move");
       return false;
    }

}

bool sendToHomePosition(){

    ROS_INFO("In getToHomePosition");

    sensor_msgs::JointState js; js.position.resize(6);
    js.position[0] = 0;
    js.position[1] = -0.1;
    js.position[2] = 0.785;
    js.position[3] = 0;
    js.position[4] = 0;
    js.position[5] = 0;

    std_msgs::Float64 msg;
    for (int id = 1; id < 6; id ++){
        msg.data = (double)js.position[id-1];
        joint_pub[id].publish( msg );
    }

    ROS_INFO("published!!! ");

    ros::Duration(3.0).sleep();

    if(hasRobotMoved(js))
        return true;
    else
        return false;
}


int main( int argc , char **argv )
{
    ros::init( argc , argv , "my_init_robot_gazebo" );

    ROS_INFO("Robot initialization");
    ros::NodeHandle nh("~");

    ros::Subscriber joint_curr_states_sub = nh.subscribe("/robotis_manipulator_h/joint_states", 5, joint_states_callback);

    ros::Rate r(10);
    joint_values.resize(6);

    joint_pub[1] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint1_position_controller/command", 1);
    joint_pub[2] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint2_position_controller/command", 1);
    joint_pub[3] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint3_position_controller/command", 1);
    joint_pub[4] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint4_position_controller/command", 1);
    joint_pub[5] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint5_position_controller/command", 1);
    joint_pub[6] = nh.advertise<std_msgs::Float64>("/robotis_manipulator_h/joint6_position_controller/command", 1);

    while(joint_curr_states_sub.getNumPublishers() == 0)
                r.sleep();

    while(joint_pub[1].getNumSubscribers() == 0)
            ros::spinOnce();

    if(!sendToHomePosition()){
        ros::spinOnce();
        //sendToHomePosition();
    }
    else
    {
        joint_curr_states_sub.shutdown();
    }

    ros::Rate rr(10);
    while(ros::ok())
    {
        rr.sleep();
        ros::spinOnce();
    }


    return 0;
}
