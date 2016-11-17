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

#include <robotis_controller_msgs/ControlTorque.h>
#include <robotis_controller_msgs/PublishPosition.h>


# define MAX_JOINT_ID 6

std::string joint_name [ MAX_JOINT_ID + 1 ] =
{ "None", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

ros::Publisher joint_states_pub;
ros::Publisher joint_pub [ MAX_JOINT_ID + 1 ];

std::vector<double> joint_values;

void joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{

    for ( int _joint_index = 0; _joint_index < msg->name.size(); _joint_index++ )
    {
        double _joint_position = msg->position[ _joint_index ];

       joint_values[_joint_index] = _joint_position;
    }

//    std::cout << " JOINT VALUES : " << std::endl;
//    for (int j=0; j< 6; j++)
//        std::cout << j << " : " << joint_values[j] << std::endl;

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

    sensor_msgs::JointState js;
    js.name.resize(6);
    js.position.resize(6);
    js.name[0] = "joint1";
    js.name[1] = "joint2";
    js.name[2] = "joint3";
    js.name[3] = "joint4";
    js.name[4] = "joint5";
    js.name[5] = "joint6";
    js.position[0] = -1.2;
    js.position[1] = -0.1;
    js.position[2] = 0.9;
    js.position[3] = 0;
    js.position[4] = 0;
    js.position[5] = 0;

    joint_states_pub.publish(js);
    ros::Duration(3.0).sleep();

    if(hasRobotMoved(js))
        return true;
    else
        return false;
}


int main( int argc , char **argv )
{
    ros::init( argc , argv , "my_init_robot" );

    ROS_INFO("In my main KInematics node");
    ros::NodeHandle nh("~");

    ros::Publisher ct_pub, pp_pub;

    ct_pub = nh.advertise<robotis_controller_msgs::ControlTorque>("/control_torque", 100);
    pp_pub = nh.advertise<robotis_controller_msgs::PublishPosition>("/publish_position", 100);

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states",1, joint_states_callback);

    //to publish all joint position
    robotis_controller_msgs::PublishPosition pp;

    for(int i = 1; i <= MAX_JOINT_ID; i++)
    {
        ROS_INFO("joint_name[%d] = %s", i, joint_name[i].c_str());
        pp.name.push_back(joint_name[i]);
        pp.publish.push_back(true);
    }

    ros::Rate r(10);
    while(pp_pub.getNumSubscribers() == 0)
        r.sleep();

    pp_pub.publish(pp);
    joint_values.resize(6);

    ROS_INFO("Set Initial Pose");

    /*---------- publisher ----------*/

    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/controller_joint_states", 1);

    // torque on
    robotis_controller_msgs::ControlTorque ct;
    ct.name.push_back("joint1");
    ct.enable.push_back(true);
    ct.name.push_back("joint2");
    ct.enable.push_back(true);
    ct.name.push_back("joint3");
    ct.enable.push_back(true);
    ct.name.push_back("joint4");
    ct.enable.push_back(true);
    ct.name.push_back("joint5");
    ct.enable.push_back(true);
    ct.name.push_back("joint6");
    ct.enable.push_back(true);

    while(ct_pub.getNumSubscribers() == 0)
        r.sleep();
    ct_pub.publish(ct);

    while(joint_state_sub.getNumPublishers() == 0)
                r.sleep();

    while(joint_states_pub.getNumSubscribers() == 0)
            ros::spinOnce();

//    if(!sendToHomePosition()){
//        ros::spinOnce();
//        //sendToHomePosition();
//    }
//    else
//    {
        joint_state_sub.shutdown();
        joint_states_pub.shutdown();
//    }

    ros::Rate rr(10);
    while(ros::ok())
    {
        rr.sleep();
        ros::spinOnce();
    }


    return 0;
}
