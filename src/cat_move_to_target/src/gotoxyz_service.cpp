#include <ros/ros.h>
#include "cat_move_to_target/GetJointValues.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "cat_move_to_target/RobotState.hpp"
//  Includes opencv to process in cv::Mat
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

# define MAX_JOINT_ID 6

std::string joint_name [ MAX_JOINT_ID + 1 ] =
{ "None", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

ros::Publisher joint_states_pub;
ros::Publisher joint_pub [ MAX_JOINT_ID + 1 ];

Eigen::VectorXd joint_values;
RobotState r ;


void joint_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{

    for ( int _joint_index = 0; _joint_index < msg->name.size(); _joint_index++ )
    {
        double _joint_position = msg->position[ _joint_index ];

       joint_values(_joint_index) = _joint_position;
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
    if ( fabs(joint_values(0) - js.position[0]) < error_allowed &
         fabs(joint_values(1) - js.position[1]) < error_allowed &
         fabs(joint_values(2) - js.position[2]) < error_allowed &
         fabs(joint_values(3) - js.position[3]) < error_allowed &
         fabs(joint_values(4) - js.position[4]) < error_allowed &
         fabs(joint_values(5) - js.position[5]) < error_allowed)
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

bool calculateJointValues(Eigen::Vector3d xyz_pos, Eigen::Vector3d zyx_angles, Eigen::VectorXd& joint_values_angles, bool sendHardcodedJoints){

    if(joint_values_angles.size() != 6)
        joint_values_angles.resize(6);

    //convert zyx_angles to rotation matrix
    std::vector<double> zyx_angles_vec;
    zyx_angles_vec.push_back(zyx_angles(0));
    zyx_angles_vec.push_back(zyx_angles(1));
    zyx_angles_vec.push_back(zyx_angles(2));
    Eigen::MatrixXd rot(3,3);
    rot = r.rotationMatrixFromZYXeuler(zyx_angles_vec);

    std::cout << "ROtation MaTrIx" << std::endl;
    std::cout << rot << std::endl;

    if(!r.InverseKinematics(xyz_pos, rot, joint_values_angles))
        return false;


    std::cout << " xyz_pos : \n"
             <<  xyz_pos << " \n ";
    std::cout << " rot : \n"
             <<  rot << " \n ";
    std::cout << " JOINTS : \n"
             <<  joint_values_angles << " \n ";

    return true;
}


bool getJointValues(cat_move_to_target::GetJointValues::Request &req,
         cat_move_to_target::GetJointValues::Response &res)
{

    Eigen::Vector3d xyz_pos;
    Eigen::Vector3d zyx_angles;
    xyz_pos(0) = req.pos_x;
    xyz_pos(1) = req.pos_y;
    xyz_pos(2) = req.pos_z;
    zyx_angles(0) = req.zyx_angle_z;
    zyx_angles(1) = req.zyx_angle_y;
    zyx_angles(2) = req.zyx_angle_x;

    Eigen::VectorXd joint_values(6);

    if (! calculateJointValues(xyz_pos, zyx_angles, joint_values, req.hardcoded_wrist) )
    {
        ROS_INFO("HERE");
        joint_values(0) = -1000; joint_values(1) = -1000; joint_values(2) = -1000;
        joint_values(3) = -1000; joint_values(4) = -1000; joint_values(5) = -1000;
    }

    res.theta1 = joint_values(0);
    res.theta2 = joint_values(1);
    res.theta3 = joint_values(2);
    res.theta4 = joint_values(3);
    res.theta5 = joint_values(4);
    res.theta6 = joint_values(5);

    ROS_INFO("request: pos_x=%3.4f, pos_y=%6.4f, pos_z=%3.4f", req.pos_x, req.pos_y, req.pos_z);
    ROS_INFO("sending back response: [%3.4f %3.4f %3.4f]", res.theta1, res.theta2, res.theta3);
    return true;
}


int main( int argc , char **argv )
{
    ros::init( argc , argv , "gotoxyz_service" );

    ros::NodeHandle nh("~");

    joint_values.resize(6);

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states",1, joint_states_callback);

    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/controller_joint_states", 1);

    ros::ServiceServer service = nh.advertiseService("getJointValues", getJointValues);
    ROS_INFO("Ready to send joint values");

    ros::spin();
    return 0;
}
