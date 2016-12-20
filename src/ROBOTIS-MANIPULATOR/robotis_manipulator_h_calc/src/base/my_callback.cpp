/*
 * callback.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

#include <control_msgs/JointControllerState.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <math.h>
#include <Eigen/Dense>

#include "../../include/state.h"

extern CalcState ManipulatorH;

extern std::string joint_name[ MAX_JOINT_ID + 1 ];

extern moveit_msgs::DisplayTrajectory moveit_msg;
extern geometry_msgs::Pose ik_msg;

extern bool task_20m_running ;

pthread_t tra_gene_20ms;

// cat new function: go to home position
void getToHomePosition()
{

    ROS_INFO("In getToHomePosition");
    sensor_msgs::JointState js;
    js.name.resize(7);
    js.position.resize(7);
    js.name[1] = "joint1";
    js.name[2] = "joint2";
    js.name[3] = "joint3";
    js.name[4] = "joint4";
    js.name[5] = "joint5";
    js.name[6] = "joint6";
    js.position[1] = -1.6;
    js.position[2] = -0.1;
    js.position[3] = 0.8;
    js.position[4] = 0.03;
    js.position[5] = 1.35;
    js.position[6] = 1.43;

    for ( int _joint_index = 0; _joint_index < js.position.size(); _joint_index++ )
    {
        std::string _joint_name = js.name[ _joint_index ];

        double _joint_position = js.position[ _joint_index ];

        for ( int _name_index = 1; _name_index < MAX_JOINT_ID + 1; _name_index++ )
        {
            if( joint_name[ _name_index ] == _joint_name )
            {
                JointState::m_goal_joint_state[ _name_index].m_position = _joint_position;
                JointState::m_real_joint_state[ _name_index ].m_position = _joint_position;

                if ( ManipulatorH.get_ini_pose == false )
                    ManipulatorH.get_ini_pose = true;

                break;
            }
        }
    }
    return;
}


void joint_curr_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
    for ( int _joint_index = 0; _joint_index < msg->name.size(); _joint_index++ )
    {
        std::string _joint_name = msg->name[ _joint_index ];
        double _joint_position = msg->position[ _joint_index ];

        for ( int _name_index = 1; _name_index < MAX_JOINT_ID + 1; _name_index++ )
        {
            if( joint_name[ _name_index ] == _joint_name )
            {
                JointState::m_curr_joint_state[ _name_index ].m_position = _joint_position;

                if ( ManipulatorH.get_ini_pose == false )
                    ManipulatorH.get_ini_pose = true;

                break;
            }
        }
    }
    return;
}

void joint_real_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
    for ( int _joint_index = 0; _joint_index < msg->name.size(); _joint_index++ )
    {
        std::string _joint_name = msg->name[ _joint_index ];
        double _joint_position = msg->position[ _joint_index ];

        for ( int _name_index = 1; _name_index < MAX_JOINT_ID + 1; _name_index++ )
        {
            if( joint_name[ _name_index ] == _joint_name )
            {
                JointState::m_real_joint_state[ _name_index ].m_position = _joint_position;

                if ( ManipulatorH.get_ini_pose == false )
                    ManipulatorH.get_ini_pose = true;

                break;
            }
        }
    }
    return;
}

void joint_fake_states_callback( const sensor_msgs::JointState::ConstPtr& msg )
{
    if ( task_20m_running == true )
    {
        ROS_INFO("Previous task is still alive!");
        return;
    }
    else
    {
        for ( int _joint_index = 0; _joint_index < msg->name.size(); _joint_index ++ )
        {
            std::string _joint_name = msg->name[ _joint_index ];
            double _joint_position = msg->position[ _joint_index ];

            for ( int _name_index = 1; _name_index < MAX_JOINT_ID + 1; _name_index++ )
            {
                if ( joint_name[ _name_index ] == _joint_name )
                {
                    JointState::m_fake_joint_state[ _name_index ].m_position = _joint_position;
                    break;
                }
            }
        }

        /* ----- for send trajectory ----- */

        ROS_INFO("[start] send trajectory");

        ManipulatorH.calc_tra = ManipulatorH.display_planned_path_positions;

        task_20m_running = true;
        ManipulatorH.cnt = 0;

    }
}

void display_planned_path_callback( const moveit_msgs::DisplayTrajectory::ConstPtr& msg )
{
    moveit_msg = *msg;

    pthread_create( &tra_gene_20ms , NULL , moveit_trajectory_proc , NULL );
}

void fk_msgs_callback( const std_msgs::Bool::ConstPtr& msg )
{
    std_msgs::Bool fk_msgs = *msg;

    ManipulatorH.solve_fk = fk_msgs.data;
}

void ik_msgs_callback( const geometry_msgs::Pose::ConstPtr& msg )
{
    ik_msg = *msg;

    pthread_create( &tra_gene_20ms , NULL , task_traejectory_proc , NULL );
}
