#include <ros/ros.h>
#include <cat_common/cloud_common.h>

#include <sensor_msgs/JointState.h>

#include <apriltags/AprilTagDetections.h>

#include "../include/cat_move_to_target/pick.hpp"
#include "cat_move_to_target/RobotState.hpp"

#include <pluginlib/class_loader.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <robotis_controller_msgs/ControlTorque.h>
#include <robotis_controller_msgs/PublishPosition.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cstdlib>
#include "cat_move_to_target/GetJointValues.h"

#define MANIPULATOR_Z_OFFSET 0.018

ros::ServiceClient client;
ros::Publisher pub_robot_joints;
RobotState robot_state_;

//geometry_msgs::PoseStamped loadPose(int getAPose){

//    std::vector<double> p_1_p(3);
//    std::vector<double> p_1_o(4);

//    std::vector<double> p_2_p(3);
//    std::vector<double> p_2_o(4);

//    std::vector<double> p_3_p(3);
//    std::vector<double> p_3_o(4);

//    std::vector<double> p_4_p(3);
//    std::vector<double> p_4_o(4);

//    //NOT WORKING
////    p_1_p[0]= 0.11;
////    p_1_p[1]= 0.29 ;
////    p_1_p[2]= 0.33;
////    p_1_o[0]= -0.56 ;
////    p_1_o[1]= 0.82 ;
////    p_1_o[2]= 0.04 ;
////    p_1_o[3]= 0.06 ;
////    // 1.2 , 0, 0.785, 0 , 1.42 , 0

//    p_1_p[0]= 0.278196;
//    p_1_p[1]= 0 ;
//    p_1_p[2]= 0.713626;
//    p_1_o[0]= 0 ;
//    p_1_o[1]= 0.382655 ;
//    p_1_o[2]= 0 ;
//    p_1_o[3]= 0.923891 ;

//    // 0 , 0, 0, 0 , 0 , 0

//    p_2_p[0]= -0.304234 ;
//    p_2_p[1]= -0.161317;
//    p_2_p[2]= 0.33619 ;
//    p_2_o[0]= 0.79135 ;
//    p_2_o[1]= -0.603053 ;
//    p_2_o[2]= -0.0557928 ;
//    p_2_o[3]= 0.0835433 ;

//    // -2.64 , 0.127804 , 0.619968 , -0.0340136 , 1.41004 , 1.80399

//    p_3_p[0]= 0.410996;
//    p_3_p[1]= 0 ;
//    p_3_p[2]= 0.45314;
//    p_3_o[0]= 0 ;
//    p_3_o[1]= 0.706966 ;
//    p_3_o[2]= 0 ;
//    p_3_o[3]= 0.707248 ;

//    // 0 , 0, 0.785, 0 , 0 , 0


////    p_4_p[0]= -0.012;
////    p_4_p[1]= -0.41082 ;
////    p_4_p[2]= 0.45314;
////    p_4_o[0]= -0.507134 ;
////    p_4_o[1]= -0.492561 ;
////    p_4_o[2]= 0.507368 ;
////    p_4_o[3]= -0.492723 ;

////    // -1.6 , 0, 0.785, 0 , 0 , 0

//    p_4_p[0]= 0.247;
//    p_4_p[1]= -0.353 ;
//    p_4_p[2]= 0.496;
//    p_4_o[0]= -0.550 ;
//    p_4_o[1]= 0.190 ;
//    p_4_o[2]= 0.281 ;
//    p_4_o[3]= 0.763 ;

//    geometry_msgs::PoseStamped pos;

//    if(getAPose == 1)
//    {
//        pos.pose.position.x = p_1_p[0];
//        pos.pose.position.y = p_1_p[1];
//        pos.pose.position.z = p_1_p[2];
//        pos.pose.orientation.x = p_1_o[0];
//        pos.pose.orientation.y = p_1_o[1];
//        pos.pose.orientation.z = p_1_o[2];
//        pos.pose.orientation.w = p_1_o[3];
//    }
//    else if (getAPose == 2)
//    {
//        pos.pose.position.x = p_2_p[0];
//        pos.pose.position.y = p_2_p[1];
//        pos.pose.position.z = p_2_p[2];
//        pos.pose.orientation.x = p_2_o[0];
//        pos.pose.orientation.y = p_2_o[1];
//        pos.pose.orientation.z = p_2_o[2];
//        pos.pose.orientation.w = p_2_o[3];
//    }
//    else if (getAPose == 3)
//    {
//        pos.pose.position.x = p_3_p[0];
//        pos.pose.position.y = p_3_p[1];
//        pos.pose.position.z = p_3_p[2];
//        pos.pose.orientation.x = p_3_o[0];
//        pos.pose.orientation.y = p_3_o[1];
//        pos.pose.orientation.z = p_3_o[2];
//        pos.pose.orientation.w = p_3_o[3];
//    }
//    else if (getAPose == 4)
//    {
//        pos.pose.position.x = p_4_p[0];
//        pos.pose.position.y = p_4_p[1];
//        pos.pose.position.z = p_4_p[2];
//        pos.pose.orientation.x = p_4_o[0];
//        pos.pose.orientation.y = p_4_o[1];
//        pos.pose.orientation.z = p_4_o[2];
//        pos.pose.orientation.w = p_4_o[3];
//    }

//    std::cout << pos << std::endl;
//    return pos;
//}

//geometry_msgs::PoseStamped correctGOTOPose(geometry_msgs::PoseStamped& pose_msg){
//     geometry_msgs::PoseStamped goto_pose;

//     goto_pose.header = pose_msg.header;
//     goto_pose.pose = pose_msg.pose;

//     goto_pose.pose.position.z += 0.2; // to be on top of the cylinder
//     goto_pose.pose.position.z -= MANIPULATOR_Z_OFFSET;

//     std::cout << "Real Goal Pose" << std::endl;
//     std::cout << goto_pose << std::endl;

//     return goto_pose;
//}

//bool listenToTagPose(geometry_msgs::PoseStamped& pose_msg, int tag_id){

//    tf::TransformListener listener;
//    tf::StampedTransform transformListen;

//    std::string tag_frame;
//    std::stringstream ss;
//    ss << "tag" <<tag_id;
//    tag_frame = ss.str();

//    geometry_msgs::TransformStamped tf_geomMsg;

//    try
//    {
////        listener.waitForTransform("end_effector", tag_frame, ros::Time(), ros::Duration(3.0));
////        listener.lookupTransform("end_effector", tag_frame, ros::Time(), transformListen);
//        listener.waitForTransform(tag_frame, "world", ros::Time(), ros::Duration(3.0));
//        listener.lookupTransform(tag_frame, "world", ros::Time(), transformListen);
//    }
//    catch(tf::TransformException &ex)
//    {
//        ROS_WARN("%s",ex.what());
//        return false;
//    }
//    tf::transformStampedTFToMsg(transformListen, tf_geomMsg);

//    std::cout << tf_geomMsg << std::endl;

//    pose_msg.pose.position.x = tf_geomMsg.transform.translation.x;
//    pose_msg.pose.position.y = tf_geomMsg.transform.translation.y;
//    pose_msg.pose.position.z = tf_geomMsg.transform.translation.z;
//    pose_msg.pose.orientation.x = tf_geomMsg.transform.rotation.x;
//    pose_msg.pose.orientation.y = tf_geomMsg.transform.rotation.y;
//    pose_msg.pose.orientation.z = tf_geomMsg.transform.rotation.z;
//    pose_msg.pose.orientation.w = tf_geomMsg.transform.rotation.w;

//    pose_msg.header.stamp = ros::Time::now();
//    pose_msg.header.frame_id = tf_geomMsg.header.frame_id;

//    pose_msg = correctGOTOPose(pose_msg);

//    return true;
//}

//bool move_robot_xyz(std::vector<double>& joint_values){

//    ROS_INFO(" Moving Robot .....");

//    robot_state_.setGoalJointStates(joint_values);

//    std::cout << robot_state_.getGoalJointStates() << std::endl;

//    if (robot_state_.checkGoalJointViability()){
//        ROS_INFO("JOINT STATE IS VIABILE");
//        //pub_robot_joints.publish(robot_state_.getGoalJointStates());
//    }
//    else
//        ROS_INFO("JOINT STATE NOT VIABILE");

//    ros::Duration(10.0).sleep();

//    double error_allowed = 0.1;
//    if ( fabs(joint_values[0] - (robot_state_.getRobotJointStates()).position[0]) < error_allowed &
//         fabs(joint_values[1] - (robot_state_.getRobotJointStates()).position[1]) < error_allowed &
//         fabs(joint_values[2] - (robot_state_.getRobotJointStates()).position[2]) < error_allowed &
//         fabs(joint_values[3] - (robot_state_.getRobotJointStates()).position[3]) < error_allowed &
//         fabs(joint_values[4] - (robot_state_.getRobotJointStates()).position[4]) < error_allowed &
//         fabs(joint_values[5] - (robot_state_.getRobotJointStates()).position[5]) < error_allowed)
//    {
//        ROS_INFO("CHECKUP : Robot Moved");
//        ros::Duration(3.0).sleep();
//        return true;
//    }
//    else
//    {
//       ROS_INFO("CHECKUP : Robot Did Not Moved");
//       return false;
//    }
//}


//bool getJoints(geometry_msgs::PoseStamped& pos)
//{
//    cat_move_to_target::GetJointValues srv;
//    srv.request.pos_x = pos.pose.position.x;
//    srv.request.pos_y = pos.pose.position.y;
//    srv.request.pos_z = pos.pose.position.z;
//    srv.request.ori_x = pos.pose.orientation.x;
//    srv.request.ori_y = pos.pose.orientation.y;
//    srv.request.ori_z = pos.pose.orientation.z;
//    srv.request.ori_w = pos.pose.orientation.w;
//    srv.request.hardcoded_wrist = true;

//    if(client.call(srv)){
//         ROS_INFO("Theta1: %3.4f", srv.response.theta1);
//         ROS_INFO("Theta2: %3.4f", srv.response.theta2);
//         ROS_INFO("Theta3: %3.4f", srv.response.theta3);
//         ROS_INFO("Theta4: %3.4f", srv.response.theta4);
//         ROS_INFO("Theta5: %3.4f", srv.response.theta5);
//         ROS_INFO("Theta6: %3.4f", srv.response.theta6);
//    }
//    else
//    {
//        ROS_ERROR("Failed to call service ");
//        return false;
//    }

//    std::vector<double> joint_values(6);
//    joint_values[0] = srv.response.theta1;
//    joint_values[1] = srv.response.theta2;
//    joint_values[2] = srv.response.theta3;
////    joint_values[3] = srv.response.theta4;
////    joint_values[4] = srv.response.theta5;
////    joint_values[5] = srv.response.theta6;

//    joint_values[3] = 1.7;
//    joint_values[4] = -1.1;
//    joint_values[5] = 0;


//    if(robot_state_.checkGoalJointViability())
//        if(move_robot_xyz(joint_values))
//            ROS_INFO(" yeah ! ");
//        else
//            ROS_INFO(" not that yeah no :( ");
//    else
//        ROS_INFO(" Did not find a viable solution");
//}


//void move_robot_callback(const sensor_msgs::JointState::ConstPtr& current_joint_states){

//    robot_state_.setRobotJointStates(current_joint_states);
//}


//bool isInHomePosition(){

//    std::vector<double> home_pos= robot_state_.getHomePosition();

//    return robot_state_.checkRobotPos(home_pos);
//}


int main(int argc, char** argv){

    ros::init(argc, argv, "move_to_target");
//    ros::NodeHandle n;

//    ros::AsyncSpinner spinner(1);
//    spinner.start();

//    client = n.serviceClient<cat_move_to_target::GetJointValues>("get_joint_values");

//    pub_robot_joints = n.advertise<sensor_msgs::JointState>("/controller_joint_states",1);

//    ros::Subscriber robot_joints_sub = n.subscribe("/joint_states",1, move_robot_callback);

//    ros::spinOnce();

//    ROS_INFO("Checking if in Home Position.... ");

//    while(!isInHomePosition){
//        ros::spinOnce();
//    }

//    ROS_INFO("Reached Home Position. Starting.... ");

//    geometry_msgs::PoseStamped pos;

//    int next = 1;

//    while (ros::ok())
//    {
//        std::cout << "Press 1-4 to go to pose, 5 to find tag and 0 to leave : ";
//        std::cin >> next;

//        if (next == 0)
//        {
//            ros::shutdown();
//        }

//        if(next == 1)
//        {

//            ROS_INFO("LOADING POSE 1");
//            pos = loadPose(1);
//            getJoints(pos);

//        }
//        else if(next == 2)
//        {
//            ROS_INFO("LOADING POSE 2");
//            pos = loadPose(2);
//            getJoints(pos);
//        }
//        else if(next == 3)
//        {
//            ROS_INFO("LOADING POSE 3");
//            pos = loadPose(3);
//            getJoints(pos);
//        }
//        else if(next == 4)
//        {
//            ROS_INFO("LOADING POSE 4");
//            pos = loadPose(4);
//            getJoints(pos);
//        }
//        else if(next == 5)
//        {
//            ROS_INFO("go after tag2");
//            int tag_id = 2;
//            if(listenToTagPose(pos, tag_id))
//            {
//                ROS_INFO("Found TAG2");
//                std::cout << pos << std::endl;
//                getJoints(pos);
//            }
//            else
//            {
//                ROS_INFO("Did not find TAG2");
//            }
//        }
//    }


   return 0;

}



































