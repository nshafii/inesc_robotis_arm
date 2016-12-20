/*
 * process.h
 *
 *  Created on: Jul 15, 2015
 *      Author: sch
 */

#ifndef PROCESS_H_
#define PROCESS_H_

#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointControllerState.h"
#include "moveit_msgs/DisplayTrajectory.h"

#include <math.h>
#include <Eigen/Dense>

void* moveit_trajectory_proc(void* arg);
void* task_traejectory_proc(void* arg);

#endif /* PROCESS_H_ */
