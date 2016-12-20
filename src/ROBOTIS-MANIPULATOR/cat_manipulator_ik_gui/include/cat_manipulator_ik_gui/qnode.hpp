/**
 * @file /include/cat_manipulator_ik_gui/qnode.hpp
 * *
 * @brief Adapted from Robotis Code
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cat_manipulator_ik_gui_QNODE_HPP_
#define cat_manipulator_ik_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "../../../../INESC_Robotis_Driver/include/INESC_Robotis_Driver/IK_Solver.hpp"

#define NUM_JOINTS 6

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cat_manipulator_ik_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

    void printJointState();
    void printEEFState(Eigen::VectorXd eef);
    void printEEFState();
    void printSTARTmessage();

	QStringListModel* loggingModel() { return &logging_model; }
    void logInfo(const std::string &msg);
    void log( const LogLevel &level, const std::string &msg );

    ////void send_fk_msgs( std_msgs::Bool msgs );
    //cat
    void send_jointposition_msgs(const sensor_msgs::JointState& goalJointStates);
    void send_grippercommand_msgs(const std_msgs::Bool& gripper_command);

    void get_joint_state_callback(const sensor_msgs::JointState::ConstPtr& current_joint_states);

    Eigen::VectorXd get_cur_joint_state();
    void set_cur_joint_state(Eigen::VectorXd js);
    void set_goal_joint_state(Eigen::VectorXd d);
    Eigen::VectorXd get_goal_joint_state();
    Eigen::Vector3d get_cur_eef_pos();
    void set_goal_eef_pos(Eigen::Vector3d d);
    Eigen::Vector3d get_cur_eef_zyxAngle();
    void set_goal_eef_zyxAngle(Eigen::Vector3d d);

    bool callFK();
    bool callFK(Eigen::VectorXd joint_states, Eigen::VectorXd &eef);
    bool callIK();



Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    //cat
    void update_joint_states(Eigen::VectorXd cur_joint_states);
    void update_eef(Eigen::VectorXd cur_eef); //pos + angles

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

//    ros::Publisher send_fk_msgs_pub;
//    ros::Subscriber end_effector_pose_sub;

    //cat

    ros::Publisher robot_joints_pub;
    ros::Publisher gripper_pub;
    ros::Subscriber robot_joints_sub;

    Eigen::VectorXd cur_joint_states_;
    Eigen::Vector3d cur_eef_pos;
    Eigen::Vector3d cur_eef_zyxAngle;

    Eigen::VectorXd goal_joint_states_;
    Eigen::Vector3d goal_eef_pos;
    Eigen::Vector3d goal_eef_zyxAngle;

    IK_Solver r ;
};

}

#endif /* cat_manipulator_ik_gui_QNODE_HPP_ */
