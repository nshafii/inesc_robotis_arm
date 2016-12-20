/**
 * @file /include/robotis_manipulator_h_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotis_manipulator_h_gui_QNODE_HPP_
#define robotis_manipulator_h_gui_QNODE_HPP_

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

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_manipulator_h_gui {

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

	QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg );

    void send_fk_msgs( std_msgs::Bool msgs );
    void end_effector_pose_callback( const geometry_msgs::Pose::ConstPtr& msgs );
    void send_ik_msgs( geometry_msgs::Pose msgs );

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    void update_curr_pos( double x , double y , double z );
    void update_curr_ori( double x , double y , double z , double w );

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Publisher send_fk_msgs_pub;
    ros::Publisher send_ik_msgs_pub;

    ros::Subscriber end_effector_pose_sub;
};

}  // namespace robotis_manipulator_h_gui

#endif /* robotis_manipulator_h_gui_QNODE_HPP_ */
