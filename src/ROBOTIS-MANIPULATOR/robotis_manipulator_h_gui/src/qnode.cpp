/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/robotis_manipulator_h_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_manipulator_h_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"robotis_manipulator_h_gui");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    // Add your ros communications here.
    send_fk_msgs_pub = n.advertise<std_msgs::Bool>("/robotis_manipulator_h/fk_msgs", 0);
    send_ik_msgs_pub = n.advertise<geometry_msgs::Pose>("/robotis_manipulator_h/ik_msgs", 0);

    end_effector_pose_sub = n.subscribe("/robotis_manipulator_h/end_effector_pose", 1, &QNode::end_effector_pose_callback, this);



    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(50);
    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::send_fk_msgs( std_msgs::Bool msgs )
{
    send_fk_msgs_pub.publish( msgs );

    log( Info , "Solve Forward Kinematics" );
}


void QNode::end_effector_pose_callback( const geometry_msgs::Pose::ConstPtr& msgs )
{
    log( Info , "End Effector Curr. Pose : " );

    std::stringstream log_msgs;

    log_msgs << " \n "
             << "curr. pos. x : "
             << msgs->position.x << " \n "
             << "curr. pos. y : "
             << msgs->position.y << " \n "
             << "curr. pos. z : "
             << msgs->position.z << " \n "
             << "curr. ori. x : "
             << msgs->orientation.x << " \n "
             << "curr. ori. y : "
             << msgs->orientation.y << " \n "
             << "curr. ori. z : "
             << msgs->orientation.z << " \n "
             << "curr. ori. w : "
             << msgs->orientation.w << " \n ";

    log( Info , log_msgs.str() );

    update_curr_pos( msgs->position.x , msgs->position.y , msgs->position.z );
    update_curr_ori( msgs->orientation.x , msgs->orientation.y , msgs->orientation.z , msgs->orientation.w );
}

void QNode::send_ik_msgs( geometry_msgs::Pose msgs )
{
    send_ik_msgs_pub.publish( msgs );

    log( Info , "Solve Inverse Kinematics" );

    log( Info , "End Effector Des. Pose : " );

    std::stringstream log_msgs;

    log_msgs << " \n "
             << "des. pos. x : "
             << msgs.position.x << " \n "
             << "des. pos. y : "
             << msgs.position.y << " \n "
             << "des. pos. z : "
             << msgs.position.z << " \n "
             << "des. ori. x : "
             << msgs.orientation.x << " \n "
             << "des. ori. y : "
             << msgs.orientation.y << " \n "
             << "des. ori. z : "
             << msgs.orientation.z << " \n "
             << "des. ori. w : "
             << msgs.orientation.w << " \n ";

    log( Info , log_msgs.str() );

}


}  // namespace robotis_manipulator_h_gui
