/*
 * callback.h
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#ifndef CALLBACK_H_
#define CALLBACK_H_

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"
#include "moveit_msgs/DisplayTrajectory.h"

void getToHomePosition();
void joint_curr_states_callback( const sensor_msgs::JointState::ConstPtr& msg );
void joint_fake_states_callback( const sensor_msgs::JointState::ConstPtr& msg );
void joint_real_states_callback( const sensor_msgs::JointState::ConstPtr& msg );

void display_planned_path_callback( const moveit_msgs::DisplayTrajectory::ConstPtr& msg );
void fk_msgs_callback( const std_msgs::Bool::ConstPtr& msg );
void ik_msgs_callback(const geometry_msgs::Pose::ConstPtr &msg );


#endif /* CALLBACK_H_ */
