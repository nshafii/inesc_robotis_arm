/*
 * calc_test.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <pthread.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <robotis_controller_msgs/ControlTorque.h>
#include <robotis_controller_msgs/PublishPosition.h>

#include "../include/state.h"

CalcState ManipulatorH = CalcState();

pthread_mutex_t mutex_flag  = PTHREAD_MUTEX_INITIALIZER;

std::string joint_name [ MAX_JOINT_ID + 1 ] =
{ "None", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

ros::Publisher joint_states_pub;
ros::Publisher joint_pub [ MAX_JOINT_ID + 1 ];
ros::Publisher end_effector_pose_pub;

moveit_msgs::DisplayTrajectory moveit_msg;
geometry_msgs::Pose ik_msg;

pthread_t	thread_20ms;
bool 		task_20m_running = false;

void getEEFstateAndPublish( const Eigen::Affine3d &curr_end_effector_state )
{
    ROS_INFO("----- Current End Effector's State -----");
    ROS_INFO("pos. x = %f", curr_end_effector_state.translation().coeff(0,0));
    ROS_INFO("pos. y = %f", curr_end_effector_state.translation().coeff(1,0));
    ROS_INFO("pos. z = %f", curr_end_effector_state.translation().coeff(2,0));

    Eigen::Matrix3d R = curr_end_effector_state.rotation();
    double roll_value = atan2( R.coeff(2,1), R.coeff(2,2) );
    double pitch_value = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) );
    double yaw_value = atan2 ( R.coeff(1,0) , R.coeff(0,0) );
    ROS_INFO("ori. roll = %f", roll_value );
    ROS_INFO("ori. pitch = %f", pitch_value );
    ROS_INFO("ori. yaw = %f", yaw_value );

    Eigen::Quaterniond curr_end_effector_Q( curr_end_effector_state.rotation() );

    geometry_msgs::Pose pose_msgs;

    pose_msgs.position.x = curr_end_effector_state.translation().coeff(0,0);
    pose_msgs.position.y = curr_end_effector_state.translation().coeff(1,0);
    pose_msgs.position.z = curr_end_effector_state.translation().coeff(2,0);

    pose_msgs.orientation.x = curr_end_effector_Q.x();
    pose_msgs.orientation.y = curr_end_effector_Q.y();
    pose_msgs.orientation.z = curr_end_effector_Q.z();
    pose_msgs.orientation.w = curr_end_effector_Q.w();

    end_effector_pose_pub.publish( pose_msgs );
}

void* thread_20ms_proc( void* arg )
{
    double temp_position[ MAX_JOINT_ID + 1 ];

    ros::Rate loop_rate( 20 );

    static robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    static robot_model::RobotModelPtr 			kinematic_model		=	robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    static robot_state::RobotStatePtr 			kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    static const robot_state::JointModelGroup* 	joint_model_group 	= 	kinematic_model->getJointModelGroup("arm");
    static const std::vector<std::string> 		&joint_names 		= 	joint_model_group->getJointModelNames();

    planning_scene::PlanningScene   planning_scene(kinematic_model);

    collision_detection::CollisionRequest   collision_request;
    collision_detection::CollisionResult    collision_result;

    std::vector<double> _joint_values, joint_values ;

    while( ros::ok() )
    {

        std::cout << task_20m_running << std::endl;

        kinematic_state->copyJointGroupPositions( joint_model_group , _joint_values );
        kinematic_state->copyJointGroupPositions( joint_model_group ,  joint_values );

        for ( int id = 1; id <= MAX_JOINT_ID; id++ )
        {
            _joint_values[ id - 1 ]	=	JointState::m_goal_joint_state[ id ].m_position;
            joint_values[ id - 1 ]	=	JointState::m_goal_joint_state[ id ].m_position;
        }

        kinematic_state->setJointGroupPositions( joint_model_group , joint_values );
        const Eigen::Affine3d &curr_end_effector_state	=	kinematic_state->getGlobalLinkTransform("end_effector");

        ManipulatorH.curr_task_p = curr_end_effector_state.translation();
        ManipulatorH.curr_task_R = curr_end_effector_state.rotation();

        Eigen::Quaterniond Q( ManipulatorH.curr_task_R );
        ManipulatorH.curr_task_QR = Q;

        /*---------- write goal position ----------*/


        if ( task_20m_running == true )
        {
            // send joint trajectory

            if ( ManipulatorH.solve_ik == false )
            {
                ROS_INFO("Solve IK is false");
                for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                {
                    JointState::m_goal_joint_state[ id ].m_position = ManipulatorH.calc_tra.coeff( ManipulatorH.cnt , id );
                    joint_values[ id - 1 ] = JointState::m_goal_joint_state[ id ].m_position;
                }

                ROS_INFO("----- FORWARD KINEMATICS -----");

                Eigen::Affine3d goal_end_effector_state	= Eigen::Affine3d::Identity();

                // translation
                goal_end_effector_state.translation() << ManipulatorH.task_tra( ManipulatorH.cnt , 0 ) ,
                        ManipulatorH.task_tra( ManipulatorH.cnt , 1 ) ,
                        ManipulatorH.task_tra( ManipulatorH.cnt , 2 ) ;

                // orientation
                double _cnt = ( double ) ManipulatorH.cnt / ( double ) ManipulatorH.all_time_steps;

                Eigen::Quaterniond new_QR = ManipulatorH.curr_task_QR.slerp( _cnt , ManipulatorH.goal_task_QR );
                goal_end_effector_state.rotate( new_QR.toRotationMatrix() );

                getEEFstateAndPublish(curr_end_effector_state );

            }
            else
            {
                ROS_INFO("----- INVERSE KINEMATICS -----");
                // inverse kinematics
                Eigen::Affine3d goal_end_effector_state	= Eigen::Affine3d::Identity();

                // translation
                goal_end_effector_state.translation() << ManipulatorH.task_tra( ManipulatorH.cnt , 0 ) ,
                        ManipulatorH.task_tra( ManipulatorH.cnt , 1 ) ,
                        ManipulatorH.task_tra( ManipulatorH.cnt , 2 ) ;

                // orientation
                double _cnt = ( double ) ManipulatorH.cnt / ( double ) ManipulatorH.all_time_steps;

                Eigen::Quaterniond new_QR = ManipulatorH.curr_task_QR.slerp( _cnt , ManipulatorH.goal_task_QR );
                goal_end_effector_state.rotate( new_QR.toRotationMatrix() );

                bool found_ik = kinematic_state->setFromIK(joint_model_group, goal_end_effector_state, 10, 0.1);

                if (found_ik)
                {

                    ROS_INFO("ik found");
                    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

                    for( int id = 1; id <= MAX_JOINT_ID; id++ )
                        JointState::m_goal_joint_state[ id ].m_position = joint_values[ id - 1 ];

                    for(int id = 1; id <= MAX_JOINT_ID; id++)
                    {
                        std::cout << "joint_name" << id << " = " << joint_values[id - 1 ] << std::endl;
                    }
                }
                else
                {
                    ROS_INFO("Did not find IK solution");

                    task_20m_running = false;
                    ManipulatorH.solve_ik  = false;

                    ManipulatorH.cnt = 0;
                }

                getEEFstateAndPublish(curr_end_effector_state );
            }
        }

        /*----- check self-collision -----*/

        robot_state::RobotState& collision_state = planning_scene.getCurrentStateNonConst();
        const robot_model::JointModelGroup* collision_model_group = collision_state.getJointModelGroup("arm");

        collision_request.group_name = "arm";

        collision_state.setJointGroupPositions( collision_model_group , joint_values );
        planning_scene.checkSelfCollision( collision_request , collision_result );


        if ( collision_result.collision == true )
        {
            ROS_INFO_STREAM( "Next state will be "
                             << ( collision_result.collision ? "in" : "not in" )
                             << " self collision" );

            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                temp_position[ id ] = _joint_values[ id - 1 ];

            task_20m_running = false;
            ManipulatorH.cnt = 0;
        }
        else
        {
            for ( int id = 1; id <= MAX_JOINT_ID; id++ )
                temp_position[ id ] = joint_values[ id - 1 ];
        }

        collision_result.clear();

        /*---------- publish goal joint state (for real robot) ----------*/

            sensor_msgs::JointState joint_msg;
            joint_msg.header.stamp = ros::Time::now();

            pthread_mutex_lock( &mutex_flag );
            for( int id = 1; id <= MAX_JOINT_ID; id++ )
            {
                joint_msg.name.push_back( joint_name[ id ] );
                joint_msg.position.push_back( temp_position[ id ] );
            }
            pthread_mutex_unlock( &mutex_flag );

            joint_states_pub.publish( joint_msg ); // publish joint state


        /*---------- initialize count number ----------*/

        loop_rate.sleep();
        ManipulatorH.cnt++;

        if ( ManipulatorH.cnt >= ManipulatorH.all_time_steps && task_20m_running == true )
        {
            ROS_INFO("[end] send trajectory");

            task_20m_running = false;
            ManipulatorH.solve_ik  = false;

            ManipulatorH.cnt = 0;
        }

        if ( ManipulatorH.solve_fk == true )
        {
            ROS_INFO("----- FORWARD KINEMATICS -----");
            /*----- Forward Kinematics -----*/

            kinematic_state->setJointGroupPositions( joint_model_group , joint_values );
            const Eigen::Affine3d &curr_end_effector_state	=	kinematic_state->getGlobalLinkTransform("end_effector");

            getEEFstateAndPublish( curr_end_effector_state );

            ROS_INFO("----- Current Joint Values -----");
            for( std::size_t id = 1; id < joint_names.size(); id++ )
                ROS_INFO("%s: %f", joint_names[ id ].c_str(), joint_values[ id - 1 ]);

            ROS_INFO("----- Current End Effector's State -----");
            ROS_INFO_STREAM( "Current Translation: \n" << curr_end_effector_state.translation() );
            ROS_INFO_STREAM( "Current Rotation: \n" << curr_end_effector_state.rotation() );

            ManipulatorH.solve_fk = false;
        }

    }

    return 0;
}


int main( int argc , char **argv )
{
    ros::init( argc , argv , "my_kinematics_node" );

    ROS_INFO("In my main KInematics node");
    ros::NodeHandle nh("~");

    ros::Publisher ct_pub, pp_pub;

    ct_pub = nh.advertise<robotis_controller_msgs::ControlTorque>("/control_torque", 10);
    pp_pub = nh.advertise<robotis_controller_msgs::PublishPosition>("/publish_position", 10);
    sleep(3.0);

    //to publish all joint position
    robotis_controller_msgs::PublishPosition pp;

    for(int i = 1; i <= MAX_JOINT_ID; i++)
    {
        ROS_INFO("joint_name[%d] = %s", i, joint_name[i].c_str());
        pp.name.push_back(joint_name[i]);
        pp.publish.push_back(true);
    }
    pp_pub.publish(pp);


    ros::Subscriber joint_real_states_sub;

   joint_real_states_sub = nh.subscribe("/move_group/fake_controller_joint_states", 5, joint_real_states_callback);

    getToHomePosition();

//    while ( ManipulatorH.get_ini_pose == false ){
//      //  ROS_INFO("TRAPPED");
//        ros::spinOnce();
//    }

    //ros::spinOnce();

    ROS_INFO("Set Initial Pose");

//    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
//    {
//        JointState::m_goal_joint_state[ id ].m_position = JointState::m_real_joint_state[ id ].m_position;
//    }

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
    ct_pub.publish(ct);

    /* moveit subscribe */
    ros::Subscriber joint_fake_states_sub 	 = nh.subscribe("/move_group/fake_controller_joint_states", 5, joint_fake_states_callback);
    ros::Subscriber display_planned_path_sub = nh.subscribe("/move_group/display_planned_path", 		5, display_planned_path_callback);

    /* from gui */
    ros::Subscriber fk_msgs_sub = nh.subscribe("/robotis_manipulator_h/fk_msgs", 1, fk_msgs_callback);
    ros::Subscriber ik_msgs_sub = nh.subscribe("/robotis_manipulator_h/ik_msgs", 1, ik_msgs_callback);

    /*from tag finder*/
    ros::Subscriber ik_tagmsgs_sub = nh.subscribe("/pose_tag_msgs", 1, ik_msgs_callback);


    end_effector_pose_pub = nh.advertise<geometry_msgs::Pose>("/robotis_manipulator_h/end_effector_pose", 1);

    /*---------- 20ms thread ----------*/

    //pthread_create(&thread_20ms, NULL, thread_20ms_proc, NULL);

    ros::spin();

    return 0;
}
