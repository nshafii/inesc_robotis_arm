/*
 * jointstate.h
 *
 *  Created on: Jul 10, 2015
 *      Author: sch
 */

#ifndef JOINTSTATE_H_
#define JOINTSTATE_H_

#include <Eigen/Dense>

#define PI	3.141592

#define MAX_JOINT_ID	6
#define MAX_ITER		5

#define deg_rad 	PI / 180.0
#define rad_deg 	180.0 / PI

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

class JointData
{
public:
	double m_position;
	double m_velocity;
	double m_effort;

	int m_p_gain;
	int m_i_gain;
	int m_d_gain;
};

class JointState
{
public:
	static JointData m_curr_joint_state[MAX_JOINT_ID + 1];
	static JointData m_goal_joint_state[MAX_JOINT_ID + 1];
	static JointData m_fake_joint_state[MAX_JOINT_ID + 1];
	static JointData m_real_joint_state[MAX_JOINT_ID + 1];
};

#endif /* JOINTSTATE_H_ */
