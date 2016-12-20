/*
 * jointstate.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: sch
 */

#include "../../include/state.h"

JointData JointState::m_goal_joint_state[ MAX_JOINT_ID + 1 ]; // goal state

JointData JointState::m_curr_joint_state[ MAX_JOINT_ID + 1 ]; // current state from gazebo
JointData JointState::m_real_joint_state[ MAX_JOINT_ID + 1 ]; // current state from real robot

JointData JointState::m_fake_joint_state[ MAX_JOINT_ID + 1 ]; // final state of sending trajectory from moveit


