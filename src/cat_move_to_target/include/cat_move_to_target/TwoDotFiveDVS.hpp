#ifndef cat_move_to_target_TWO_DOT_FIVE_D_VS_HPP
#define cat_move_to_target_TWO_DOT_FIVE_D_VS_HPP

/*-----------------------------------------*
 * Includes
 *-----------------------------------------*/
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include "cat_move_to_target/RobotState.hpp"
#include <sensor_msgs/JointState.h>

#define epsilon 0.0000000001
#define A1 0.159
#define A2 sqrt(0.264*0.264+0.030*0.030)
#define A3 0.030
#define A4 0.258
/*-----------------------------------------*
 * Definition
 *-----------------------------------------*/


/*****************************************************************************
** Class [SIMPLE_TAG]
*****************************************************************************/
class TwoDotFiveDVS{


public:

    TwoDotFiveDVS();

//    //falta fazer set do ori
//    void set_p_xyz(std::vector<double> xyz);
//    void set_pStar_xyz(std::vector<double> xyz);

//    // ------------------------------------------------------------------------------
//    //                                LINEAR VELOCITY only
//    // ------------------------------------------------------------------------------
//    bool set_error_vl();
//    void setInteractionMatrix_vl();
//    bool setCameraVelocity_vl();
//    bool setEEFvelFromCameraVelocities_vl();
//    void setJacobian_vl();
//    bool setQdot_vl();

//    bool setJointAnglesToSend_vl();
//    std::vector<double> getJointAnglesToSend_vl();

//    // ------------------------------------------------------------------------------
//    //                          LINEAR VELOCITY + ANGULAR VELOCITY
//    // ------------------------------------------------------------------------------

//    bool set_error();
//    void setInteractionMatrix();
//    bool setCameraVelocity();
//    bool setEEFvelFromCameraVelocities();
//    void setJacobian();
//    bool setQdot();

//    // ------------------------------------------------------------------------------
//    void set_R_ec(Eigen::MatrixXd r);
//    void set_R_ec_star(Eigen::MatrixXd r);
//    void set_R_0c(Eigen::MatrixXd r);
//    void set_d_ec(Eigen::VectorXd d);
//    void set_joints(std::vector<double> joints);
//    void setLambda(double l);

//    bool do_calculus();
//    bool do_calculus_vl();

//    Eigen::MatrixXd RotationFromUnitQuaternion(std::vector<double> q);

//    //TODO write a function to update the positions in jointstate_


private:

//    //! Interaction matrix
//    // 6X6
//    Eigen::MatrixXd L;
//    Eigen::MatrixXd Lv;
//    Eigen::MatrixXd Lw;

//    //! Error
//    // 6X1
//    Eigen::VectorXd error;

//    //! Jacobian
//    // 6X6
//    Eigen::MatrixXd J;

//    //! Jacobian Inverse
//    // 6X6
//    Eigen::MatrixXd Ji;

//    //! current feature
//    // 3X1
//    Eigen::VectorXd p_xyz;
//    Eigen::VectorXd s_xyz;

//    //! desired feature
//    // 3X1
//    Eigen::VectorXd pStar_xyz;
//    Eigen::VectorXd sStar_xyz;

//    //! desired feature
//    // 4X1
//    Eigen::VectorXd theta_u;

//    //! joint velocities
//    // 6X1
//    Eigen::VectorXd q_dot;

//    //! camera velocities
//    // 6X1
//    Eigen::VectorXd cam_v;
//    //! eef velocities
//    // 6X1
//    Eigen::VectorXd eef_v;

//    //! controller law step
//    double lambda;

//    //! M matrix: conversion from camera to eef velocities
//    // 6X6
//    Eigen::MatrixXd M;

//    //! Rotation from eef to camera
//    // 3x3
//    Eigen::MatrixXd R_ec;
//    Eigen::MatrixXd R_ec_star;

//    //! Rotation from world to camera
//    // 3x3
//    Eigen::MatrixXd R_0c;

//    //! translation from eef to camera
//    // 3X1
//    Eigen::VectorXd d_ec;

//    //! robot state
//    //TODO
//    RobotState robotstate_;

//    //!joint states
//    std::vector<double> joint_state_;
//    std::vector<double> joint_angles_to_send;
//    std::vector<double> joint_angles_received;
//    //flags
//    bool is_set_L;
//    bool is_set_error;
//    bool is_set_J;
//    bool is_set_Ji;
//    bool is_set_s_xyz;
//    bool is_set_sStar_xyz;
//    bool is_set_theta_u;
//    bool is_set_q_dot;
//    bool is_set_cam_v;
//    bool is_set_eef_v;
//    bool is_set_lambda;
//    bool is_set_M;
//    bool is_set_R_ec;
//    bool is_set_R_ec_star;
//    bool is_set_R_0c;
//    bool is_set_d_ec;
//    bool is_set_robotstate_;
//    bool is_set_joint_angles_to_send;

//    //TODO write rest of flags


    //*********************
    //private functions
    //*********************
//    Eigen::Matrix3d sk3x1(std::vector<double> p);
//    double trace_of_matrix(Eigen::MatrixXd r);

//    bool setMmatrix();
//    bool setMmatrix_vl();
//    bool setThetaU();
//    bool setJacobianInverse();
//    void putFlagsDown();
//    void set_s_xyz();
//    void set_sStar_xyz();


//    Eigen::MatrixXd rotationAroundZ(double theta);

//    Eigen::MatrixXd rotationAroundX(double theta);

//    Eigen::MatrixXd rotationAroundY(double theta);


//    //bool getJointVectorFromJointState(std::vector<double> joints);



};
#endif
