#ifndef IK_SOLVER_H
#define IK_SOLVER_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include <sensor_msgs/JointState.h>

/*****************************************************************************
** DEFINITIONS
*****************************************************************************/

#define JOINT1_UPPER_LIMIT (M_PI-0.02)
#define JOINT1_LOWER_LIMIT (-M_PI+0.02)

#define JOINT2_UPPER_LIMIT 1.37
#define JOINT2_LOWER_LIMIT -0.84

#define JOINT3_UPPER_LIMIT 1.48
#define JOINT3_LOWER_LIMIT -M_PI/2

#define JOINT4_UPPER_LIMIT M_PI
#define JOINT4_LOWER_LIMIT -M_PI

#define JOINT5_UPPER_LIMIT 1.55
#define JOINT5_LOWER_LIMIT -1.55

#define JOINT6_UPPER_LIMIT M_PI
#define JOINT6_LOWER_LIMIT -M_PI


#define X_W_RB (0.009+0.009/2.0)
#define Y_W_RB (0.009+0.009/2.0)
#define Z_W_RB (0.006+0.017)

#define Z_RBJ2 0.159

#define X_J2J3 0.030
#define Z_J2J3 0.264

#define X_J3J4 0.258
#define Z_J3J4 0.030

#define X_J6EEF (0.645-0.258-0.264)

/*****************************************************************************
** Class [ROBOT_STATE]
*****************************************************************************/

class IK_Solver{

public:

    IK_Solver();
    ~IK_Solver(){}
    void setRobotJointStates(const sensor_msgs::JointState::ConstPtr& current_joint_states);
    sensor_msgs::JointState getRobotJointStates();

    void setGoalJointStates(const sensor_msgs::JointState::ConstPtr& current_joint_states);
    void setGoalJointStates(std::vector<double> joint_values);
    sensor_msgs::JointState getGoalJointStates();

    std::vector<double> getHomePosition();

    double shortestAngleDiff(double target, double source);

    bool checkRobotPos(std::vector<double> goal_joints);
    bool checkGoalJointViability();
    bool checkGoalJointViabilityAndCorrect();
    //Eigen::MatrixXd getJacobianW(std::vector<double> joint_values);

    void vectorToPoseStamped(std::vector<double> pos, geometry_msgs::PoseStamped& pose);

    void quaternionToRotation(Eigen::Matrix3d& R, double q1, double q2, double q3, double q0);
    Eigen::MatrixXd rpy2rotation( double r, double p, double y );
    Eigen::Quaterniond rpy2quaternion( double r, double p, double y );

    //bool InverseKinematics_old(Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles);
    //bool ForwardKinematics_old(Eigen::VectorXd joint_angles, Eigen::Vector3d &xyz_eef, Eigen::MatrixXd &rot_eef);
    bool InverseKinematics(Eigen::VectorXd current_joint_angles,Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles);
    bool ForwardKinematics(Eigen::VectorXd joint_angles, Eigen::Vector3d &xyz_eef, Eigen::MatrixXd &rot_eef);
    void findSingularities(Eigen::VectorXd joint_angles, Eigen::MatrixXd &J, bool &is_sing_J11, bool &is_sing_J22, Eigen::MatrixXd &J11, Eigen::MatrixXd &J22);


    Eigen::MatrixXd rotationMatrixFromUnitQuaternion(std::vector<double> q);
    std::vector<double> rotationMatrixToUnitQuaternion(Eigen::MatrixXd r);

    void rotationMatrixToZYXeuler(Eigen::MatrixXd rot, Eigen::Vector3d &zyx_angle);
    Eigen::MatrixXd rotationMatrixFromZYXeuler(std::vector<double> zyx_eulerAngles);

    void rotationAroundZ(double theta, Eigen::MatrixXd &rotation);
    void rotationAroundX(double theta, Eigen::MatrixXd &rotation);
    void rotationAroundY(double theta, Eigen::MatrixXd &rotation);

private:

    //functions
    Eigen::MatrixXd dh_transformation(Eigen::Vector4d dh_parameters);
    //var
    sensor_msgs::JointState joint_states_;
    sensor_msgs::JointState goal_joint_states_;
    std::vector<double> home_position_joints_;

    double normalizeAngle(double t);
    double roundToNdecimalPlaces( double x , int n);

    Eigen::MatrixXd computeBaseToSphWristPoint_old(double theta1, double theta2, double theta3);
    void computeBaseToSphWristPoint(double theta1, double theta2, double theta3, Eigen::Vector3d &pos, Eigen::MatrixXd &rot);
    double my_normAngle(double alpha);
};


Eigen::MatrixXd rx( double s );
Eigen::MatrixXd ry( double s );
Eigen::MatrixXd rz( double s );
bool my_isnan(double x);
Eigen::Matrix3d transpose3DMatrix(Eigen::Matrix3d& input);

#endif // IK_SOLVER_H

