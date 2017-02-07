#include <ros/ros.h>

#include "INESC_Robotis_Driver/SetToolPose.h"
#include "INESC_Robotis_Driver/SetToolPosition.h"
#include "INESC_Robotis_Driver/HomeArm.h"
#include "INESC_Robotis_Driver/GetToolPose.h"
#include "INESC_Robotis_Driver/SetToolPosition.h"
#include "INESC_Robotis_Driver/GetJointValues.h"
#include "INESC_Robotis_Driver/Stop.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "../../INESC_Robotis_Driver/include/INESC_Robotis_Driver/IK_Solver.hpp"
#include "../../INESC_Robotis_Driver/include/INESC_Robotis_Driver/TrajectoryGenerator.hpp"

# define MAX_JOINT_ID 6

std::string joint_name[ MAX_JOINT_ID + 1] = { "None", "joint1", "joint2",
    "joint3", "joint4", "joint5", "joint6" };

ros::Publisher joint_states_pub;
ros::Publisher joint_pub[ MAX_JOINT_ID + 1];

Eigen::VectorXd joint_values;
IK_Solver r;

Eigen::Vector3d cur_eef_pos;

Eigen::Vector3d cur_eef_zyxAngle;

Eigen::Vector3d xyz_pos;
Eigen::Vector3d R_P_Y_angles;

Eigen::Vector3d local_eef_pos;
Eigen::Vector3d goal_eef_pos;

TrajectoryGenerator tragectoryGenrator;

double linearVelocity;

bool stop = true;

bool inGoalPose = false;

bool bigRotationChange = false;

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) {

  for (int _joint_index = 0; _joint_index < msg->name.size(); _joint_index++) {
    double _joint_position = msg->position[_joint_index];

    joint_values(_joint_index) = _joint_position;
  }
}

bool calculateJointValues(Eigen::VectorXd current_joint_angles,
    Eigen::Vector3d xyz_pos, Eigen::Vector3d roll_pitch_yaw_angles,
    Eigen::VectorXd& joint_values_angles, bool sendHardcodedJoints) {

  if (joint_values_angles.size() != 6)
    joint_values_angles.resize(6);

  //convert euler_angles (x,y,z), to  rotation matrix

  Eigen::Matrix3d rollAngle(3, 3);
  rollAngle << 1, 0, 0, 0, cos(roll_pitch_yaw_angles.x()), -sin(
      roll_pitch_yaw_angles.x()), 0, sin(roll_pitch_yaw_angles.x()), cos(
      roll_pitch_yaw_angles.x());

  Eigen::Matrix3d pitchAngle(3, 3);

  pitchAngle << cos(roll_pitch_yaw_angles.y()), 0, sin(
      roll_pitch_yaw_angles.y()), 0, 1, 0, -sin(roll_pitch_yaw_angles.y()), 0, cos(
      roll_pitch_yaw_angles.y());

  Eigen::Matrix3d yawAngle(3, 3);
  yawAngle << cos(roll_pitch_yaw_angles.z()), -sin(roll_pitch_yaw_angles.z()), 0, sin(
      roll_pitch_yaw_angles.z()), cos(roll_pitch_yaw_angles.z()), 0, 0, 0, 1;

  Eigen::Matrix3d rot = rollAngle * pitchAngle * yawAngle;

  std::cout << "Rotation Matirx" << std::endl;
  std::cout << rot << std::endl;

  if (!r.InverseKinematics(current_joint_angles, xyz_pos, rot,
      joint_values_angles))
    return false;

  std::cout << " xyz_pos : \n" << xyz_pos << " \n ";
  std::cout << " rot : \n" << rot << " \n ";
  std::cout << " JOINTS : \n" << joint_values_angles << " \n ";

  return true;
}

bool homeArm(INESC_Robotis_Driver::HomeArm::Request &req,
    INESC_Robotis_Driver::HomeArm::Response &res) {
  stop = true;
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  int num_joints = 6;
  msg.name.resize(num_joints);
  msg.position.resize(num_joints);

  msg.name[0] = "joint1";
  msg.name[1] = "joint2";
  msg.name[2] = "joint3";
  msg.name[3] = "joint4";
  msg.name[4] = "joint5";
  msg.name[5] = "joint6";

  msg.position[0] = 0;
  msg.position[1] = 0;
  msg.position[2] = 0;
  msg.position[3] = 0;
  msg.position[4] = 0;
  msg.position[5] = 0;
  joint_states_pub.publish(msg);

  return true;
}

bool stopArm(INESC_Robotis_Driver::Stop::Request &req,
    INESC_Robotis_Driver::Stop::Response &res) {
  stop = true;
}

bool getToolPos(INESC_Robotis_Driver::GetToolPose::Request &req,
    INESC_Robotis_Driver::GetToolPose::Response &res) {

  Eigen::MatrixXd rot_tool(3, 3);
  Eigen::Vector3d xyz_pos;
  Eigen::Vector3d R_P_Y_tool;
  r.ForwardKinematics(joint_values, xyz_pos, rot_tool);

  r.rotationMatrixToZYXeuler(rot_tool, R_P_Y_tool);

  res.pos_x = xyz_pos(0) ;
  res.pos_y = xyz_pos(1) ;
  res.pos_z = xyz_pos(2) ;

  res.zyx_angle_z= R_P_Y_tool(0);
  res.zyx_angle_y= R_P_Y_tool(1);
  res.zyx_angle_x = R_P_Y_tool(2);
}

bool getJointValues(INESC_Robotis_Driver::GetJointValues::Request &req,
    INESC_Robotis_Driver::GetJointValues::Response &res) {

  res.theta1 = joint_values(0);
  res.theta2 = joint_values(1);
  res.theta3 = joint_values(2);
  res.theta4 = joint_values(3);
  res.theta5 = joint_values(4);
  res.theta6 = joint_values(5);

}


bool setToolPose(INESC_Robotis_Driver::SetToolPose::Request &req,
    INESC_Robotis_Driver::SetToolPose::Response &res) {

  inGoalPose = false;

  xyz_pos(0) = req.pos_x;
  xyz_pos(1) = req.pos_y;
  xyz_pos(2) = req.pos_z;
  if(fabs(R_P_Y_angles(0)- req.Roll_angle_x) > 0.2 or fabs(R_P_Y_angles(1)- req.Pitch_angle_y) > 0.2
      or fabs(R_P_Y_angles(2)- req.Yaw_angle_z) > 0.2)
    bigRotationChange=true;
  else
    bigRotationChange=false;

  R_P_Y_angles(0) = req.Roll_angle_x;
  R_P_Y_angles(1) = req.Pitch_angle_y;
  R_P_Y_angles(2) = req.Yaw_angle_z;

  if (req.linear_velocity > 0.12)
    linearVelocity = 0.12;
  else if (req.linear_velocity < 0.0001)
    linearVelocity = 0.000001;
  else
    linearVelocity = req.linear_velocity;

  Eigen::VectorXd eff_joint_values(6);

  if (!calculateJointValues(joint_values, xyz_pos, R_P_Y_angles,
      eff_joint_values, req.linear_velocity)) {
    ROS_INFO("The Trajectory is not in Feasible Area");
    stop = true;
    res.result = 0;
    goal_eef_pos = cur_eef_pos;
  } else {
    stop = false;
    goal_eef_pos = xyz_pos;
    res.result = 1;
  }

  ROS_INFO("request: pos_x=%3.4f, pos_y=%6.4f, pos_z=%3.4f", req.pos_x,
      req.pos_y, req.pos_z);
  ROS_INFO("sending back response: [%d]", res.result);

  return true;
}

bool setToolPosition(INESC_Robotis_Driver::SetToolPosition::Request &req,
    INESC_Robotis_Driver::SetToolPosition::Response &res) {
  inGoalPose = false;

  xyz_pos(0) = req.pos_x;
  xyz_pos(1) = req.pos_y;
  xyz_pos(2) = req.pos_z;
  bigRotationChange=false;

  if (req.linear_velocity > 0.12)
    linearVelocity = 0.12;
  else if (req.linear_velocity < 0.0001)
    linearVelocity = 0.01;
  else
    linearVelocity = req.linear_velocity;

  Eigen::VectorXd eff_joint_values(6);

  if (!calculateJointValues(joint_values, xyz_pos, R_P_Y_angles,
      eff_joint_values, req.linear_velocity)) {
    ROS_INFO("The Trajectory is not in Feasible Area");
    stop = true;
    res.result = 0;
    goal_eef_pos = cur_eef_pos;
  } else {
    stop = false;
    goal_eef_pos = xyz_pos;
    res.result = 1;
  }

  ROS_INFO("request: pos_x=%3.4f, pos_y=%6.4f, pos_z=%3.4f", req.pos_x,
      req.pos_y, req.pos_z);
  ROS_INFO("sending back response: [%d]", res.result);

  return true;
}

bool callFK() {
  // update the current eef pose.
  Eigen::MatrixXd cur_rot_eef(3, 3);
  r.ForwardKinematics(joint_values, cur_eef_pos, cur_rot_eef);

  r.rotationMatrixToZYXeuler(cur_rot_eef, cur_eef_zyxAngle);

  return true;
}

bool callIK() {

  //convert euler_angles (x,y,z), to  rotation matrix
  ROS_INFO("Calling IK! ");

  Eigen::Matrix3d rollAngle(3, 3);
  rollAngle << cos(R_P_Y_angles.x()), -sin(R_P_Y_angles.x()), 0, sin(
      R_P_Y_angles.x()), cos(R_P_Y_angles.x()), 0, 0, 0, 1;

  Eigen::Matrix3d pitchAngle(3, 3);

  pitchAngle << cos(R_P_Y_angles.y()), 0, sin(R_P_Y_angles.y()), 0, 1, 0, -sin(
      R_P_Y_angles.y()), 0, cos(R_P_Y_angles.y());

  Eigen::Matrix3d yawAngle(3, 3);
  yawAngle << cos(R_P_Y_angles.z()), -sin(R_P_Y_angles.z()), 0, sin(
      R_P_Y_angles.z()), cos(R_P_Y_angles.z()), 0, 0, 0, 1;

  Eigen::Matrix3d rot = rollAngle * pitchAngle * yawAngle;

  std::cout << "Rotation Matrix" << std::endl;
  std::cout << rot << std::endl;

  Eigen::VectorXd eff_joint_values(6);

  if (r.InverseKinematics(joint_values, local_eef_pos, rot, eff_joint_values)) {
    ROS_INFO(" IK is possible");

    if (r.checkGoalJointViabilityAndCorrect()) {
      ROS_INFO("The calculated joint values are aout of bound");
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    int num_joints = 6;
    msg.name.resize(num_joints);
    msg.position.resize(num_joints);

    msg.name[0] = "joint1";
    msg.name[1] = "joint2";
    msg.name[2] = "joint3";
    msg.name[3] = "joint4";
    msg.name[4] = "joint5";
    msg.name[5] = "joint6";

    msg.position = r.getGoalJointStates().position;
    joint_states_pub.publish(msg);
    return true;
  } else {
    ROS_INFO(" IK is not possible to achieve!!");
    return false;
  }

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "GoToPose_service");

  ros::NodeHandle nh("~");

  R_P_Y_angles(0) =0;
  R_P_Y_angles(1) =1.57;
  R_P_Y_angles(2) =3.14;

  joint_values.resize(6);

  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1,
      joint_states_callback);

  joint_states_pub = nh.advertise<sensor_msgs::JointState>(
      "/controller_joint_states", 1);

  ros::ServiceServer service = nh.advertiseService("setToolPose", setToolPose);

  ros::ServiceServer setPositionservice = nh.advertiseService("setToolPosition", setToolPosition);

  ros::ServiceServer homeArmService = nh.advertiseService("homeArm", homeArm);

  ros::ServiceServer stopService = nh.advertiseService("stopArm", stopArm);

  ros::ServiceServer getToolPoseService = nh.advertiseService("getToolPos", getToolPos);

  ros::ServiceServer getJointValuesService = nh.advertiseService("getJointValues", getJointValues);

  int rate = 100;
  int internalRate = 1;
  double refresh_rate = 0.25; // works with very fast speed
  ros::Rate loop_rate(rate);
  double beginTime;

  while (ros::ok()) {
    double secs = ros::Time::now().toSec();

    nh.setParam("/GoToPose_service/inGoalPose", inGoalPose);

    ros::spinOnce();

    callFK();

    std::cout << "time " << secs << std::endl;

    if (!stop)
    {

      if (internalRate == 1)
      {
        if(bigRotationChange)
        {
          double duration = (goal_eef_pos - cur_eef_pos).norm() / 0.00001;
          beginTime = secs;

          ROS_INFO("in time %f trajectory is created for the duration %f ", secs,
            duration);
          tragectoryGenrator.setLinear(cur_eef_pos, goal_eef_pos, duration);

          std::cout << "cur eef pos: " << cur_eef_pos << std::endl;
          std::cout << "goal Pos: " << goal_eef_pos << std::endl;

          std::cout << "goal_curr_dist:" << (goal_eef_pos - cur_eef_pos).norm()
              << std::endl;
          std::cout << "===============================" << std::endl;
        }
        else
        {
          double duration = (goal_eef_pos - cur_eef_pos).norm() / linearVelocity;
          beginTime = secs;

          ROS_INFO("in time %f trajectory is created for the duration %f ", secs,
            duration);
          tragectoryGenrator.setLinear(cur_eef_pos, goal_eef_pos, duration);

          std::cout << "cur eef pos: " << cur_eef_pos << std::endl;
          std::cout << "goal Pos: " << goal_eef_pos << std::endl;

          std::cout << "goal_curr_dist:" << (goal_eef_pos - cur_eef_pos).norm()
              << std::endl;
          std::cout << "===============================" << std::endl;
        }

      }

      double currTime = secs - beginTime;

      double stopTimeForRotation = 2; // it was 3
      if(currTime > stopTimeForRotation){
        bigRotationChange =false;
      }

      local_eef_pos = tragectoryGenrator.getLinearPosition(currTime);
      std::cout << "for the currtime: " << currTime << " local Pos: "
          << local_eef_pos << std::endl;

      if ((goal_eef_pos - cur_eef_pos).norm() <= 0.01)
      {
        ROS_INFO("========THE ROBOT IS IN GOAL POS");

        local_eef_pos = goal_eef_pos;
        std::cout << "goal_curr_dist" << (goal_eef_pos - cur_eef_pos).norm()
            << std::endl;
        std::cout << "local_curr_dist" << (local_eef_pos - cur_eef_pos).norm()
            << std::endl;

        std::cout << "cur eef pos: " << cur_eef_pos << std::endl;
        std::cout << "goal Pos: " << goal_eef_pos << std::endl;
        std::cout << "local Pos: " << local_eef_pos << std::endl;

        inGoalPose = true;
      }

      ROS_INFO("========THE ROBOT IS MOVING");

      callIK();

      internalRate++;

      if (internalRate > (rate / refresh_rate)) {
        internalRate = 1;
      }
    }
    else {
      inGoalPose = true;
      internalRate = 1;
      ROS_INFO("====== THE ROBOT IS STOPPED");
    }
    loop_rate.sleep();

  }

  return 0;
}
