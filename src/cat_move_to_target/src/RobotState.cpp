#include "../include/cat_move_to_target/RobotState.hpp"

#include <iostream>

/*****************************************************************************
** Implementation [Robot_state]
*****************************************************************************/

RobotState::RobotState()
{
    joint_states_.position.resize(6);
    joint_states_.name.resize(6);

    joint_states_.name[0] = "joint1";
    joint_states_.name[1] = "joint2";
    joint_states_.name[2] = "joint3";
    joint_states_.name[3] = "joint4";
    joint_states_.name[4] = "joint5";
    joint_states_.name[5] = "joint6";

    joint_states_.position[0] = 0;
    joint_states_.position[1] = 0;
    joint_states_.position[2] = 0;
    joint_states_.position[3] = 0;
    joint_states_.position[4] = 0;
    joint_states_.position[5] = 0;

    goal_joint_states_.position.resize(6);
    goal_joint_states_.name.resize(6);

    goal_joint_states_.name[0] = "joint1";
    goal_joint_states_.name[1] = "joint2";
    goal_joint_states_.name[2] = "joint3";
    goal_joint_states_.name[3] = "joint4";
    goal_joint_states_.name[4] = "joint5";
    goal_joint_states_.name[5] = "joint6";

    goal_joint_states_.position[0] = 0;
    goal_joint_states_.position[1] = 0;
    goal_joint_states_.position[2] = 0;
    goal_joint_states_.position[3] = 0;
    goal_joint_states_.position[4] = 0;
    goal_joint_states_.position[5] = 0;

    home_position_joints_.resize(6);
    home_position_joints_[0] = -0.6;
    home_position_joints_[1] = -0.1;
    home_position_joints_[2] = 0.8;
    home_position_joints_[3] = 0.03;
    home_position_joints_[4] = 1.35;
    home_position_joints_[5] = 1.43;
}

sensor_msgs::JointState RobotState::getRobotJointStates()
{
    return joint_states_;
}

void RobotState::setRobotJointStates(const sensor_msgs::JointState::ConstPtr& current_joint_states)
{
 joint_states_.header = current_joint_states->header;
 joint_states_.position = current_joint_states->position;
 joint_states_.name = current_joint_states->name;
}

sensor_msgs::JointState RobotState::getGoalJointStates()
{
    return goal_joint_states_;
}

void RobotState::setGoalJointStates(const sensor_msgs::JointState::ConstPtr& goal_joint_states)
{
 goal_joint_states_.header = goal_joint_states->header;
 goal_joint_states_.position = goal_joint_states->position;
 goal_joint_states_.name = goal_joint_states->name;
}

void RobotState::setGoalJointStates(std::vector<double> joint_values)
{
    goal_joint_states_.header.stamp = ros::Time::now();

    goal_joint_states_.position[0] = joint_values[0];
    goal_joint_states_.position[1] = joint_values[1];
    goal_joint_states_.position[2] = joint_values[2];
    goal_joint_states_.position[3] = joint_values[3];
    goal_joint_states_.position[4] = joint_values[4];
    goal_joint_states_.position[5] = joint_values[5];
}

std::vector<double> RobotState::getHomePosition(){
    return home_position_joints_;
}

bool RobotState::checkRobotPos(std::vector<double> goal_joints){

    ROS_INFO (" Verifying robot position ... ");

    ros::spinOnce();

    std::cout << " JOINT VALUES : " << std::endl;
    for (int j=0; j< 6; j++)
        std::cout << j << " : " << goal_joints[j] << std::endl;

    double error_allowed = 0.1;
    if ( fabs(goal_joints[0] - joint_states_.position[0]) < error_allowed &
         fabs(goal_joints[1] - joint_states_.position[1]) < error_allowed &
         fabs(goal_joints[2] - joint_states_.position[2]) < error_allowed &
         fabs(goal_joints[3] - joint_states_.position[3]) < error_allowed &
         fabs(goal_joints[4] - joint_states_.position[4]) < error_allowed &
         fabs(goal_joints[5] - joint_states_.position[5]) < error_allowed)
    {
        ROS_INFO("CHECKUP : In Goal Position");
        return true;
    }
    else
    {
       ROS_INFO("CHECKUP : Not In Goal Position");
       return false;
    }

}

bool my_isnan(double x) {
    return x != x;
}

bool RobotState::checkGoalJointViability()
{
    if( my_isnan(goal_joint_states_.position[0]) || goal_joint_states_.position[0] < JOINT1_LOWER_LIMIT || goal_joint_states_.position[0] > JOINT1_UPPER_LIMIT)
        return false;

    if( my_isnan(goal_joint_states_.position[1]) ||goal_joint_states_.position[1] < JOINT2_LOWER_LIMIT || goal_joint_states_.position[1] > JOINT2_UPPER_LIMIT)
        return false;

    if( my_isnan(goal_joint_states_.position[2]) || goal_joint_states_.position[2] < JOINT3_LOWER_LIMIT || goal_joint_states_.position[2] > JOINT3_UPPER_LIMIT)
        return false;

    if( my_isnan(goal_joint_states_.position[3]) || goal_joint_states_.position[3] < JOINT4_LOWER_LIMIT || goal_joint_states_.position[3] > JOINT4_UPPER_LIMIT)
        return false;

    if( my_isnan(goal_joint_states_.position[4]) || goal_joint_states_.position[4] < JOINT5_LOWER_LIMIT || goal_joint_states_.position[4] > JOINT5_UPPER_LIMIT)
        return false;

    if( my_isnan(goal_joint_states_.position[5]) || goal_joint_states_.position[5] < JOINT6_LOWER_LIMIT || goal_joint_states_.position[5] > JOINT6_UPPER_LIMIT)
        return false;

    return true;
}

/*
//Eigen::MatrixXd RobotState::getJacobianW(std::vector<double> joint_values){

//    //TODO : check this !

//    std::vector<double> joint1_dh(4), joint2_dh(4), joint3_dh(4), joint4_dh(4), joint5_dh(4), joint6_dh(4);
//    joint1_dh[0] = DH_J1_A;
//    joint1_dh[1] = DH_J1_ALPHA;
//    joint1_dh[2] = DH_J1_D;
//    joint1_dh[3] = joint_values[0];

//    joint2_dh[0] = DH_J2_A;
//    joint2_dh[1] = DH_J2_ALPHA;
//    joint2_dh[2] = DH_J2_D;
//    joint2_dh[3] = joint_values[1] - OFFSET_JOINT2;

//    joint3_dh[0] = DH_J3_A;
//    joint3_dh[1] = DH_J3_ALPHA;
//    joint3_dh[2] = DH_J3_D;
//    joint3_dh[3] = joint_values[2] - OFFSET_JOINT3;

//    joint4_dh[0] = DH_J4_A;
//    joint4_dh[1] = DH_J4_ALPHA;
//    joint4_dh[2] = DH_J4_D;
//    joint4_dh[3] = joint_values[3];

//    joint5_dh[0] = DH_J5_A;
//    joint5_dh[1] = DH_J5_ALPHA;
//    joint5_dh[2] = DH_J5_D;
//    joint5_dh[3] = joint_values[4];

//    joint6_dh[0] = DH_J6_A;
//    joint6_dh[1] = DH_J6_ALPHA;
//    joint6_dh[2] = DH_J6_D;
//    joint6_dh[3] = joint_values[5];

//    Eigen::MatrixXd m_j1(4, 4), m_j2(4, 4), m_j3(4, 4), m_j4(4, 4), m_j5(4, 4), m_j6(4, 4) ;

//    dh_transformation(joint1_dh, m_j1);
//    dh_transformation(joint2_dh, m_j2);
//    dh_transformation(joint3_dh, m_j3);
//    dh_transformation(joint4_dh, m_j4);
//    dh_transformation(joint5_dh, m_j5);
//    dh_transformation(joint6_dh, m_j6);

//    Eigen::MatrixXd m_01(4, 4), m_02(4, 4), m_03(4, 4), m_04(4, 4), m_05(4, 4), m_06(4, 4), jacobian_w(3,6);
//    m_01 = m_j1;
//    m_02 = m_01 * m_j2;
//    m_03 = m_02 * m_j3;
//    m_04 = m_03 * m_j4;
//    m_05 = m_04 * m_j5;
//    m_06 = m_05 * m_j6;

//    std::cout << "m_01 " << std::endl << m_01 << std::endl;
//    std::cout << "m_02 " << std::endl << m_02 << std::endl;
//    std::cout << "m_03 " << std::endl << m_03 << std::endl;
//    std::cout << "m_04 " << std::endl << m_04 << std::endl;
//    std::cout << "m_05 " << std::endl << m_05 << std::endl;
//    std::cout << "m_06 " << std::endl << m_06 << std::endl;


//    jacobian_w << m_01(0,2) , m_02(0,2), m_03(0,2), m_04(0,2), m_05(0,2), m_06(0,2),
//            m_01(1,2) , m_02(1,2), m_03(1,2), m_04(1,2), m_05(1,2), m_06(1,2),
//            m_01(2,2) , m_02(2,2), m_03(2,2), m_04(2,2), m_05(2,2), m_06(2,2);

//    return jacobian_w;

//}
*/

void RobotState::vectorToPoseStamped(std::vector<double> pos, geometry_msgs::PoseStamped& pose){
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x= pos[0];
    pose.pose.position.y= pos[1];
    pose.pose.position.z= pos[2];
    pose.pose.orientation.x= pos[3];
    pose.pose.orientation.y= pos[4];
    pose.pose.orientation.z= pos[5];
    pose.pose.orientation.w= pos[6];
}

Eigen::Matrix3d transpose3DMatrix(Eigen::Matrix3d& input){
    Eigen::Matrix3d output;

    output(0,0) = input(0,0);
    output(0,1) = input(1,0);
    output(0,2) = input(2,0);

    output(1,0) = input(0,1);
    output(1,1) = input(1,1);
    output(1,2) = input(2,1);

    output(2,0) = input(0,2);
    output(2,1) = input(1,2);
    output(2,2) = input(2,2);

    return output;
}


// -------------------------------------------------------------------------------------------------------------
//                                              ANGLES
// -------------------------------------------------------------------------------------------------------------

void RobotState::quaternionToRotation(Eigen::Matrix3d &R, double q1, double q2, double q3, double q0){

    R(0,0)  = q0*q0  +  q1*q1  -  q2*q2  -  q3*q3;
    R(0,1)  = 2 * (q1*q2  -  q0*q3);
    R(0,2)  = 2 * (q1*q3  +  q0*q2);

    R(1,0)  = 2 * (q1*q2  +  q0*q3);
    R(1,1)  = q0*q0  -  q1*q1  +  q2*q2  -  q3*q3;
    R(1,2)  = 2 * (q2*q3  -  q0*q1);

    R(2,0)  = 2 * (q1*q3  -  q0*q2);
    R(2,1)  = 2 * (q2*q3  +  q0*q1);
    R(2,2)  = q0*q0  -  q1*q1  -  q2*q2  +  q3*q3;
}

Eigen::MatrixXd rx( double s )
{
    Eigen::MatrixXd R(3,3);

    R << 1.0, 	 0.0, 	  0.0,
         0.0, cos(s), -sin(s),
         0.0, sin(s),  cos(s);

    return R;
}

Eigen::MatrixXd ry( double s )
{
    Eigen::MatrixXd R(3,3);

    R << cos(s), 0.0, sin(s),
         0.0, 	 1.0, 	 0.0,
        -sin(s), 0.0, cos(s);

    return R;
}

Eigen::MatrixXd rz( double s )
{
    Eigen::MatrixXd R(3,3);

    R << cos(s), -sin(s), 0.0,
         sin(s),  cos(s), 0.0,
            0.0,     0.0, 1.0;

    return R;
}

Eigen::MatrixXd RobotState::rpy2rotation( double r, double p, double y )
{
    Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

    return R;
}

Eigen::Quaterniond RobotState::rpy2quaternion( double r, double p, double y )
{
    Eigen::Matrix3d R = rpy2rotation(r, p, y);

    Eigen::Matrix3d R_plus;
    R_plus = R.block(0,0,3,3);

    Eigen::Quaterniond QR;
    QR = R_plus;

    return QR;
}

void RobotState::rotationAroundZ(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0,          0,          1;

}

void RobotState::rotationAroundX(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << 1.0, 	 0.0, 	  0.0,
            0.0, cos(theta), -sin(theta),
            0.0, sin(theta),  cos(theta);

}

void RobotState::rotationAroundY(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << cos(theta), 0.0, sin(theta),
            0.0, 	 1.0, 	 0.0,
            -sin(theta), 0.0, cos(theta);
}

void RobotState::rotationMatrixToZYXeuler(Eigen::MatrixXd rot, Eigen::Vector3d &zyx_angle)
{
    double beta = atan2(-rot(2,0), sqrt(rot(0,0)*rot(0,0)+rot(1,0)*rot(1,0)));
    double alpha = atan2(rot(1,0)/cos(beta), rot(0,0)/cos(beta));
    double gamma = atan2(rot(2,1)/cos(beta), rot(2,2)/cos(beta));

    double z_angle = alpha;
    double y_angle = beta;
    double x_angle = gamma;

    if(zyx_angle.size()!=3)
        zyx_angle.resize(3);
    zyx_angle(0) = z_angle;
    zyx_angle(1) = y_angle;
    zyx_angle(2) = x_angle;

}

Eigen::MatrixXd RobotState::rotationMatrixFromZYXeuler(std::vector<double> zyx_eulerAngles)
{
    //TODO check this
    double alpha = zyx_eulerAngles[0];
    double beta = zyx_eulerAngles[1];
    double gamma = zyx_eulerAngles[2];

    double c_a = cos(alpha); double s_a = sin(alpha);
    double c_b = cos(beta); double s_b = sin(beta);
    double c_g = cos(gamma); double s_g = sin(gamma);

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  c_a*c_b     , c_a*s_b*s_g-s_a*c_g   , c_a*s_b*c_g+s_a*s_g,
                        s_a*c_b     , s_a*s_b*s_g+c_a*c_g   , s_a*s_b*c_g-c_a*s_g,
                        -s_b        , c_b*s_g               , c_b*c_g;

    return rotation_matrix;
}

Eigen::MatrixXd RobotState::rotationMatrixFromUnitQuaternion(std::vector<double> q)
{
    //TODO check this
    double e0 = q[0];
    double e1 = q[1];
    double e2 = q[2];
    double e3 = q[3];

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  1-2*(e2*e2+e3*e3) , 2*(e1*e2-e0*e3)     , 2*(e1*e3+e0*e2),
                        2*(e1*e2+e0*e3)   , 1-2*(e1*e1+e3*e3)   , 2*(e2*e3-e0*e1),
                        2*(e1*e3-e0*e2)   , 2*(e2*e3+e0*e1)     , 1-2*(e1*e1+e2*e2);

    return rotation_matrix;
}

std::vector<double> RobotState::rotationMatrixToUnitQuaternion(Eigen::MatrixXd r)
{
    //TODO check this
    double r11 = r(0,0); double r12 = r(0,1); double r13 = r(0,2);
    double r21 = r(1,0); double r22 = r(1,1); double r23 = r(1,2);
    double r31 = r(2,0); double r32 = r(2,1); double r33 = r(2,2);

    double e0 = 1/2*sqrt(1+r11+r22+r33);
    double e1 = (r32-r23)/(4*e0);
    double e2 = (r13-r31)/(4*e0);
    double e3 = (r21-r12)/(4*e0);

    std::vector<double> q;
    q.push_back(e0);
    q.push_back(e1);
    q.push_back(e2);
    q.push_back(e3);
   return q;
}

double RobotState::roundToNdecimalPlaces( double x , int n)
{
    const double sd = pow(10, n);
    double a  =int(x*sd + (x<0? -0.5 : 0.5))/sd;
    return a;
}

double RobotState::normalizeAngle(double t)
{
    double t_;

    t_ = fmod(t+M_PI, 2*M_PI) + -M_PI;

    return t_;
}

double RobotState::my_normAngle(double alpha)
{
        // btw -pi and pi
    double angle_;
    bool is_bigger_than_pi;
    if(alpha <= M_PI & alpha > -M_PI)
        angle_ = alpha;
    else
    {
        if(alpha > M_PI)
            is_bigger_than_pi = true;
        else
            is_bigger_than_pi = false;

        while(fabs(alpha) >= M_PI)
        {
            if(is_bigger_than_pi)
                alpha = alpha - 2*M_PI;
            else
                alpha = alpha + 2*M_PI;
        }

        angle_ = alpha;
    }
    return angle_;
}

// -------------------------------------------------------------------------------------------------------------
//                                              KINEMATICS
// -------------------------------------------------------------------------------------------------------------

bool RobotState::ForwardKinematics(Eigen::VectorXd joint_angles, Eigen::Vector3d &xyz_eef, Eigen::MatrixXd &rot_eef){

    double A1 = 0.159;
    double A2 = sqrt(0.264*0.264+0.030*0.030);
    double OFFSET_JOINT2 = (M_PI/2-atan2(0.030,0.264));
    double OFFSET_JOINT3  = (M_PI/4 +atan2(0.030,0.264));
    double AE = (0.645-0.258-0.264);

    double DH_J1_A =0;
    double DH_J1_ALPHA= -M_PI/2;
    double DH_J1_D =A1;

    double DH_J2_A =A2;
    double DH_J2_ALPHA= 0;
    double DH_J2_D= 0;

    double DH_J3_A= 0.030;
    double DH_J3_ALPHA= -M_PI/2;
    double DH_J3_D =0;

    double DH_J4_A= 0;
    double DH_J4_ALPHA= M_PI/2;
    double DH_J4_D =0.258;

    double DH_J5_A= 0;
    double DH_J5_ALPHA= -M_PI/2;
    double DH_J5_D= 0;

    double DH_J6_A= 0;
    double DH_J6_ALPHA= 0;
    double DH_J6_D= AE;

    Eigen::Vector4d joint1_dh, joint2_dh, joint3_dh, joint4_dh, joint5_dh, joint6_dh;
    joint1_dh << DH_J1_A, DH_J1_ALPHA, DH_J1_D, joint_angles(0);

    joint2_dh << DH_J2_A, DH_J2_ALPHA, DH_J2_D, joint_angles(1) - OFFSET_JOINT2;

    joint3_dh << DH_J3_A, DH_J3_ALPHA, DH_J3_D, joint_angles(2) - OFFSET_JOINT3;

    joint4_dh << DH_J4_A, DH_J4_ALPHA, DH_J4_D, joint_angles(3);

    joint5_dh << DH_J5_A, DH_J5_ALPHA, DH_J5_D, joint_angles(4);

    joint6_dh << DH_J6_A, DH_J6_ALPHA, DH_J6_D, joint_angles(5);

    Eigen::MatrixXd m_j1(4,4), m_j2(4,4), m_j3(4,4), m_j4(4,4), m_j5(4,4), m_j6(4,4);
    m_j1 = dh_transformation(joint1_dh);
    m_j2 = dh_transformation(joint2_dh);
    m_j3 = dh_transformation(joint3_dh);
    m_j4 = dh_transformation(joint4_dh);
    m_j5 = dh_transformation(joint5_dh);
    m_j6 = dh_transformation(joint6_dh);

    Eigen::MatrixXd m_final(4,4);
    m_final = m_j1 * m_j2 * m_j3 * m_j4 * m_j5 * m_j6;

    // eef pose
    xyz_eef << m_final(0,3), m_final(1,3), m_final(2,3);

    rot_eef = m_final.block(0,0,3,3);

//    std::cout << "....... In Forward Kinematics ....... " << std::endl;
//    std::cout << "m_final : " << std::endl << m_final << std::endl;
//    std::cout << "xyz_eef : " << std::endl << xyz_eef << std::endl;
//    std::cout << "rot_eef : " << std::endl << rot_eef << std::endl;
//    std::cout << "..................................... " << std::endl;

}

void RobotState::computeBaseToSphWristPoint(double theta1, double theta2, double theta3, Eigen::Vector3d &pos, Eigen::MatrixXd &rot){

    double A1 = 0.159;
    double A2 = sqrt(0.264*0.264+0.030*0.030);
    double OFFSET_JOINT2 = (M_PI/2-atan2(0.030,0.264));
    double OFFSET_JOINT3  = (M_PI/4 +atan2(0.030,0.264));

    double DH_J1_A =0;
    double DH_J1_ALPHA= -M_PI/2;
    double DH_J1_D =A1;

    double DH_J2_A =A2;
    double DH_J2_ALPHA= 0;
    double DH_J2_D= 0;

    double DH_J3_A= 0.030;
    double DH_J3_ALPHA= -M_PI/2;
    double DH_J3_D =0;

    Eigen::MatrixXd spWrist_Tmatrix(4,4);
    spWrist_Tmatrix <<  1 , 0 , 0 , 0,
                        0 , 1 , 0 , 0,
                        0 , 0 , 1 , 0.258,
                        0 , 0 , 0 , 1;

    Eigen::Vector4d joint1_dh, joint2_dh, joint3_dh;
    joint1_dh << DH_J1_A, DH_J1_ALPHA, DH_J1_D, theta1;

    joint2_dh << DH_J2_A, DH_J2_ALPHA, DH_J2_D, theta2 - OFFSET_JOINT2;

    joint3_dh << DH_J3_A, DH_J3_ALPHA, DH_J3_D, theta3 - OFFSET_JOINT3;

    Eigen::MatrixXd m_j1(4,4), m_j2(4,4), m_j3(4,4);
    m_j1 = dh_transformation(joint1_dh);
    m_j2 = dh_transformation(joint2_dh);
    m_j3 = dh_transformation(joint3_dh);


    Eigen::MatrixXd m_final(4,4);
    m_final = m_j1 * m_j2 * m_j3 * spWrist_Tmatrix;

    // eef pose
    pos << m_final(0,3), m_final(1,3), m_final(2,3);

    rot = m_final.block(0,0,3,3);


//    std::cout << "... In computeBaseToSphWristPoint ... " << std::endl;
//    std::cout << "m_final : " << std::endl << m_final << std::endl;
//    std::cout << "pos : " << std::endl << pos << std::endl;
//    std::cout << "rot : " << std::endl << rot << std::endl;
//    std::cout << "..................................... " << std::endl;

}

bool RobotState::InverseKinematics(Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles){

    double l1 = 0.159 ;
    double l2 = sqrt(X_J2J3*X_J2J3 + Z_J2J3*Z_J2J3);
    double l3 = sqrt(X_J3J4*X_J3J4 + Z_J3J4*Z_J3J4);
    double l6 = 0.645-X_J3J4-Z_J2J3;
    double beta = atan(0.030/0.258);
    double gamma = atan(0.030/0.264);

    // ------- Construct Transformation matrix from 0 to 6
    Eigen::MatrixXd R06(3,3); R06 = rot;

    Eigen::MatrixXd t06(4,4);
    t06 <<  R06, pos,
            0, 0, 0, 1;

    // ------- SPHERICAL WRIST POINT DETERMINATION
    double xc = t06(0,3) - (l6 * t06(0,2));
    double yc = t06(1,3) - (l6 * t06(1,2));
    double zc = t06(2,3) - (l6 * t06(2,2));
    std::cout << "xc : " << xc << " , yc : " << yc << " , zc : " << zc << std::endl;

    // ------- Compute theta1 -> theta3
    double r = sqrt(xc*xc+yc*yc);
    double s = zc-l1;

    double A = l2;
    double B = l3;
    double C = sqrt(r*r + s*s);

    double ab, ac;

    //    std::cout << " ----------------- " << std::endl;
    //    std::cout << " C : " << C << std::endl;
    //    std::cout << " A+B : " << A+B  << std::endl;
    //    std::cout << " ----------------- " << std::endl;

    if(C <= A + B)
    {

        // cossine law : c² = a² + b² -2*a*b*cos(a^b)

        ab = 2*atan2(1, -sqrt((2*A*B+C*C-A*A-B*B)/(2*A*B-C*C+A*A+B*B)))- M_PI;
        ac = 2*atan2(1, -sqrt((2*A*C+B*B-A*A-C*C)/(2*A*C-B*B+A*A+C*C)))- M_PI;

        // compute virtual robot angles
        double theta1_linha = atan2(yc,xc);
        double theta2_linha = atan2(s,r) + ac;
        double theta3_linha = ab - M_PI/2;

        // compute real robot angles
        double theta1_robot, theta2_robot, theta3_robot;
        theta1_robot = theta1_linha;
        theta2_robot = M_PI/2 - theta2_linha - gamma ;
        theta3_robot = -theta3_linha + gamma + beta;

        if (roundToNdecimalPlaces(ac,5) == roundToNdecimalPlaces(gamma, 5))
        {
            theta2_robot = M_PI/2 - theta2_linha +gamma;
            theta3_robot = -theta3_linha -gamma -beta;
        }

        // calculus of theta4, theta5 and theta6
        double theta4, theta5, theta6;
        double theta4_robot, theta5_robot, theta6_robot;

        Eigen::MatrixXd R03(3,3);
        Eigen::Vector3d t03;
        computeBaseToSphWristPoint(theta1_robot, theta2_robot, theta3_robot+M_PI/4, t03, R03);

        Eigen::MatrixXd r_matrix(3,3);
        r_matrix = (R03).transpose()*R06;

        for (int i=0; i< r_matrix.rows(); i++)
            for(int j=0; j < r_matrix.cols(); j++)
                r_matrix(i,j) = roundToNdecimalPlaces(r_matrix(i,j),4);

        double r11 = r_matrix(0,0); double r13 = r_matrix(0,2); double r31 = r_matrix(2,0);
        double r23 = r_matrix(1,2); double r32 = r_matrix(2,1); double r33 = r_matrix(2,2);
        if(roundToNdecimalPlaces(r11,5) == 1)
        {
            theta5 = 0;
            theta4 = 0;
            theta6 = 0;
            theta5_robot =  theta5;
            theta4_robot =  theta4;
            theta6_robot =  theta6;
        }
        else if(roundToNdecimalPlaces(r11,5) == -1)
        {
            theta5 = M_PI;
            theta4 = 0;
            theta6 = 0;
            theta5_robot =  theta5;
            theta4_robot =  theta4;
            theta6_robot =  theta6;
        }
        else
        {
            theta5 = atan2( r33, -sqrt(1-r33*r33));
            theta4 = atan2(-r13, -r23);
            theta6 = atan2( r31, -r32);

            theta5_robot = -M_PI/2 - theta5;
            theta4_robot =  M_PI/2 - theta4;
            theta6_robot =  M_PI/2 - theta6;
        }

        theta6_robot = my_normAngle(theta6_robot);
        if(roundToNdecimalPlaces(r11,5) != 1 & roundToNdecimalPlaces(r11,5) != -1)
            theta5_robot = M_PI - my_normAngle(theta5_robot);
        else
            theta5_robot = my_normAngle(theta5_robot);
        theta4_robot = my_normAngle(theta4_robot);

        joint_angles << theta1_robot, theta2_robot, theta3_robot,
                        theta4_robot, theta5_robot, theta6_robot;


        //        std::cout << "...... In Inverse KInematics ....... " << std::endl;
        //        std::cout << "ab " << ab << std::endl;
        //        std::cout << "ac " << ac << std::endl;

        //        std::cout << "theta2_linha " << theta2_linha << std::endl;
        //        std::cout << "theta3_linha " << theta3_linha << std::endl;
        //        std::cout << "theta1_robot " << theta1_robot << std::endl;
        //        std::cout << "theta2_robot " << theta2_robot << std::endl;
        //        std::cout << "theta3_robot " << theta3_robot << std::endl;

        //        std::cout << "R03 " << std::endl <<R03 << std::endl;
        //        std::cout << "t03 " << std::endl <<t03 << std::endl;

        //        std::cout << "r_matrix " << std::endl <<r_matrix << std::endl;

        //        std::cout << "theta4_robot " << theta4_robot << std::endl;
        //        std::cout << "theta5_robot " << theta5_robot << std::endl;
        //        std::cout << "theta6_robot " << theta6_robot << std::endl;


        //        std::cout << "joint_angles " << std::endl <<joint_angles << std::endl;
        //        std::cout << "..................................... " << std::endl;


        return true;
    }
    else
    {
        std::cout << " XYZ given is not possible to reach." << std::endl;
        return false;
    }
}

Eigen::MatrixXd RobotState::dh_transformation(Eigen::Vector4d dh_parameters){
    double a = dh_parameters(0);
    double alpha =  dh_parameters(1);
    double d = dh_parameters(2);
    double theta = dh_parameters(3);

    Eigen::MatrixXd m(4,4);
    m <<  cos(theta),     -sin(theta)*cos(alpha),     sin(theta)*sin(alpha),      a*cos(theta),
          sin(theta),     cos(theta)*cos(alpha),      -cos(theta)*sin(alpha),     a*sin(theta),
          0,              sin(alpha),                 cos(alpha),                 d,
          0,              0,                          0,                          1;

    return m;
}

void RobotState::findSingularities(Eigen::VectorXd joint_angles, Eigen::MatrixXd &J, bool &is_sing_J11, bool &is_sing_J22,
                       Eigen::MatrixXd &J11, Eigen::MatrixXd &J22){

    double A1 = 0.159;
    double A2 = sqrt(0.264*0.264+0.030*0.030);
    double A3 = sqrt(0.258*0.258+0.030*0.030);
    double OFFSET_JOINT2 = (M_PI/2-atan2(0.030,0.264));
    double OFFSET_JOINT3  = (M_PI/4 +atan2(0.030,0.264));
    double AE = (0.645-0.258-0.264);

    double DH_J1_A =0;
    double DH_J1_ALPHA= -M_PI/2;
    double DH_J1_D =A1;

    double DH_J2_A =A2;
    double DH_J2_ALPHA= 0;
    double DH_J2_D= 0;

    double DH_J3_A= 0.030;
    double DH_J3_ALPHA= -M_PI/2;
    double DH_J3_D =0;

    double DH_J4_A= 0;
    double DH_J4_ALPHA= M_PI/2;
    double DH_J4_D =0.258;

    double DH_J5_A= 0;
    double DH_J5_ALPHA= -M_PI/2;
    double DH_J5_D= 0;

    double DH_J6_A= 0;
    double DH_J6_ALPHA= 0;
    double DH_J6_D= AE;

    Eigen::Vector4d joint1_dh, joint2_dh, joint3_dh, joint4_dh, joint5_dh, joint6_dh;
    joint1_dh << DH_J1_A, DH_J1_ALPHA, DH_J1_D, joint_angles(0);

    joint2_dh << DH_J2_A, DH_J2_ALPHA, DH_J2_D, joint_angles(1) - OFFSET_JOINT2;

    joint3_dh << DH_J3_A, DH_J3_ALPHA, DH_J3_D, joint_angles(2) - OFFSET_JOINT3;

    joint4_dh << DH_J4_A, DH_J4_ALPHA, DH_J4_D, joint_angles(3);

    joint5_dh << DH_J5_A, DH_J5_ALPHA, DH_J5_D, joint_angles(4);

    joint6_dh << DH_J6_A, DH_J6_ALPHA, DH_J6_D, joint_angles(5);

    Eigen::MatrixXd m_j1(4,4), m_j2(4,4), m_j3(4,4), m_j4(4,4), m_j5(4,4), m_j6(4,4);
    m_j1 = dh_transformation(joint1_dh);
    m_j2 = dh_transformation(joint2_dh);
    m_j3 = dh_transformation(joint3_dh);
    m_j4 = dh_transformation(joint4_dh);
    m_j5 = dh_transformation(joint5_dh);
    m_j6 = dh_transformation(joint6_dh);

    Eigen::MatrixXd m_01(4,4), m_02(4,4), m_03(4,4), m_04(4,4), m_05(4,4), m_06(4,4);
    m_01 = m_j1 ;
    m_02 = m_01 * m_j2;
    m_03 = m_02 * m_j3;
    m_04 = m_03 * m_j4;
    m_05 = m_04 * m_j5;
    m_06 = m_05 * m_j6;

    double s1 = sin(joint_angles(0)); double c1 = cos(joint_angles(0));
    double s2 = sin(joint_angles(1) - OFFSET_JOINT2); double c2 = cos(joint_angles(1) - OFFSET_JOINT2);
    double s3 = sin(joint_angles(2) - OFFSET_JOINT3); double c3 = cos(joint_angles(2) - OFFSET_JOINT3);
    double s4 = sin(joint_angles(3)); double c4 = cos(joint_angles(3));
    double s5 = sin(joint_angles(4)); double c5 = cos(joint_angles(4));
    double l3a = DH_J3_A;
    double l3b = DH_J4_D;

    Eigen::Vector3d z0, z1, z2, z3, z4, z5, z6;
    z0 << 0, 0, 1;
    z1 << -s1, c1, 0;
    z2 << -s1, c1, 0;
    z3 << -c1*c2*s3-c1*s2*c3,
        -s1*c2*s3-s1*s2*c3,
        s2*s3-c2*c3;
    z4 << s4*(c1*c2*c3-c1*s2*s3)-c4*s1,
        s4*(s1*c2*c3-s1*s2*s3)+c4*c1,
        s4*(-s2*c3-c2*s3);
    z5 << -s5*(c4*(c1*c2*c3-c1*s2*s3)+s1*s4) + c5*z3(0),
        -s5*(c4*(s1*c2*c3-s1*s2*s3)-s4*c1) + c5*z3(1),
        -s5*(c4*(-s2*c3-c2*s3))+c5*z3(2);
    z6 << z5(0), z5(1), z5(2);

    Eigen::Vector3d o0, o1, o2, o3, o4, o5, o6;
    o0 << 0, 0, 0;
    o1 << 0, 0, A1;
    o2 << c1*c2*A2, s1*c2*A2, -A2*s2+A1;
    o3 << c1*c2*c3*l3a-c1*s2*s3*l3a+c1*A2*c2,
          s1*c2*c3*l3a-s1*s2*s3*l3a+A2*c2*s1,
          -s2*c3*l3a-c2*s3*l3a-A2*s2+A1;
    o4 << l3b*z3(0) + o3(0),
          l3b*z3(1) + o3(1),
          l3b*z3(2) + o3(2);
    o5 = o4;
    // o6 << AE*z5(0) + o5(0),
    //     AE*z5(1) + o5(1),
    //     AE*z5(2) + o5(2);
    o6 = o5;

    Eigen::MatrixXd Jv(3,6), Jw(3,6);
    Jv << z0.cross(o6-o0),
          z1.cross(o6-o1),
          z2.cross(o6-o2),
          z3.cross(o6-o3),
          z4.cross(o6-o4),
          z5.cross(o6-o5);

    Jw << z0, z1, z2, z3, z4, z5;

    J << Jv, Jw;

    Eigen::MatrixXd m_0c(4,4), m_3c(4,4);
    m_3c << 1 , 0 , 0 , 0 ,
            0 , 1 , 0 , 0 ,
            0 , 0 , 1 , l3b,
            0 , 0 , 0 , 1;
    m_0c = m_03 * m_3c;
    Eigen::Vector3d oc = m_0c.block(0,3, 3,1);

    J11 << z0.cross(oc-o0),
        z1.cross(oc-o1),
        z2.cross(oc-o2);

    J22 << z3 , z4 , z5;

    if(roundToNdecimalPlaces(J11.determinant(), 6) == 0)
        is_sing_J11 = true;
    else
        is_sing_J11 = false;

    if(roundToNdecimalPlaces(J22.determinant(), 6) == 0)
        is_sing_J22 = true;
    else
        is_sing_J22 = false;


    // prints
    std::cout << "........ In find sigularities ....... " << std::endl;
    std::cout << "J : " << std::endl << J << std::endl;
    std::cout << "J11 : " << std::endl << J11 << std::endl;
    std::cout << "J22 : " << std::endl << J22 << std::endl;
    std::cout << "is_sing_J11 : " << std::endl << is_sing_J11 << std::endl;
    std::cout << "is_sing_J22 : " << std::endl << is_sing_J22 << std::endl;
    std::cout << "..................................... " << std::endl;

}

// -------------------------------------------------------------------------------------------------------------
//                                              OLD
// -------------------------------------------------------------------------------------------------------------
bool RobotState::ForwardKinematics_old(Eigen::VectorXd joint_angles, Eigen::Vector3d &xyz_eef, Eigen::MatrixXd &rot_eef){

    double ROBOT_0_OFFSET = -M_PI/4;

    if(joint_angles.size() != 6)
    {
        std::cout << "Number of joint angles must be 6!" << std::endl;
        return false;
    }

    //set theta angles
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2] +ROBOT_0_OFFSET;
    double theta4 = joint_angles[3];
    double theta5 = joint_angles[4];
    double theta6 = joint_angles[5];


    std::cout << "theta1 : " << theta1 << std::endl;
    std::cout << "theta2 : " << theta2 << std::endl;
    std::cout << "theta3 : " << theta3 << std::endl;
    std::cout << "theta4 : " << theta4 << std::endl;
    std::cout << "theta5 : " << theta5 << std::endl;
    std::cout << "theta6 : " << theta6 << std::endl;


    // MATRICES

    Eigen::MatrixXd eye3(3,3); eye3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd eye4(4,4); eye4 = Eigen::Matrix4d::Identity();
    Eigen::Vector3d zeros_; zeros_<< 0, 0, 0;

    // ---- WORLD -> ROBOT_BASE
    Eigen::MatrixXd R_world_robotBase(3,3); R_world_robotBase = eye3;
    Eigen::Vector3d T_world_robotBase; T_world_robotBase << X_W_RB, Y_W_RB, Z_W_RB;
    Eigen::MatrixXd M_world_robotBase(4,4);
    M_world_robotBase << R_world_robotBase, T_world_robotBase,
            zeros_.transpose(), 1;

    std::cout << " M_world_robotBase " << std::endl;
    std::cout << M_world_robotBase << std::endl;

    // ---- ROTATION J1
    Eigen::MatrixXd R_j1(3,3); rotationAroundZ(theta1, R_j1);
    Eigen::MatrixXd M_rot_j1(4,4);
    M_rot_j1 << R_j1, zeros_,
            zeros_.transpose(), 1;

    std::cout << " M_rot_j1 " << std::endl;
    std::cout << M_rot_j1 << std::endl;

    // ---- ROBOT_BASE -> J2
    Eigen::MatrixXd R_robotBase_j2(3,3);  R_robotBase_j2= eye3;
    Eigen::Vector3d T_robotBase_j2;  T_robotBase_j2 << 0, 0, Z_RBJ2;
    Eigen::MatrixXd M_robotBase_j2(4,4);
    M_robotBase_j2 << R_robotBase_j2, T_robotBase_j2,
            zeros_.transpose(), 1;

    std::cout << " M_robotBase_j2 " << std::endl;
    std::cout << M_robotBase_j2 << std::endl;

    // ---- ROTATION J2
    Eigen::MatrixXd R_j2(3,3); rotationAroundY(theta2, R_j2);
    Eigen::MatrixXd M_rot_j2(4,4);
    M_rot_j2 << R_j2, zeros_,
            zeros_.transpose(), 1;

    // ---- J2 -> J3
    Eigen::MatrixXd R_j2_j3(3,3); R_j2_j3 = eye3;
    Eigen::Vector3d T_j2_j3; T_j2_j3  << X_J2J3, 0, Z_J2J3;
    Eigen::MatrixXd M_j2_j3(4,4);
    M_j2_j3 << R_j2_j3, T_j2_j3,
                zeros_.transpose(), 1;

    std::cout << " M_j2_j3 " << std::endl;
    std::cout << M_j2_j3 << std::endl;

    // ---- ROTATION J3
    Eigen::MatrixXd R_j3(3,3); rotationAroundY(theta3, R_j3);
    Eigen::MatrixXd M_rot_j3(4,4);
    M_rot_j3 << R_j3, zeros_,
        zeros_.transpose(), 1;

    std::cout << " M_rot_j3 " << std::endl;
    std::cout << M_rot_j3 << std::endl;

    // ---- J3 -> J4 (spherical wrist mid point)
    Eigen::MatrixXd R_j3_j4(3,3); R_j3_j4 = eye3;
    Eigen::Vector3d T_j3_j4; T_j3_j4 << X_J3J4, 0, Z_J3J4;
    Eigen::MatrixXd M_j3_j4(4,4);
    M_j3_j4 << R_j3_j4, T_j3_j4,
            zeros_.transpose(), 1;

    std::cout << " M_j3_j4 " << std::endl;
    std::cout << M_j3_j4 << std::endl;

    // ---- ROTATION J4
    Eigen::MatrixXd R_j4(3,3); rotationAroundX(theta4, R_j4);
    Eigen::MatrixXd M_rot_j4(4,4);
    M_rot_j4 << R_j4, zeros_,
            zeros_.transpose(), 1;
    // ---- ROTATION J5
    Eigen::MatrixXd R_j5(3,3); rotationAroundY(theta5, R_j5);
    Eigen::MatrixXd M_rot_j5(4,4);
    M_rot_j5 << R_j5, zeros_,
            zeros_.transpose(), 1;
    // ---- ROTATION J6
    Eigen::MatrixXd R_j6(3,3); rotationAroundX(theta6, R_j6);
    Eigen::MatrixXd M_rot_j6(4,4);
    M_rot_j6 << R_j6, zeros_,
            zeros_.transpose(), 1;
    // ---- J6 -> eef
    Eigen::MatrixXd R_j6_eef(3,3); R_j6_eef = eye3;
    Eigen::Vector3d T_j6_eef; T_j6_eef<< X_J6EEF, 0, 0;
    Eigen::MatrixXd M_j6_eef(4,4);
    M_j6_eef << R_j6_eef, T_j6_eef,
            zeros_.transpose(), 1;

    std::cout << " M_rot_j4 " << std::endl;
    std::cout << M_rot_j4 << std::endl;
    std::cout << " M_rot_j5 " << std::endl;
    std::cout << M_rot_j5 << std::endl;
    std::cout << " M_rot_j6 " << std::endl;
    std::cout << M_rot_j6 << std::endl;
    std::cout << " M_j6_eef " << std::endl;
    std::cout << M_j6_eef << std::endl;

    // -------------------------
    // FORWARD KIN MATRIX

    Eigen::MatrixXd m_fk(4,4);
    m_fk = M_world_robotBase * M_rot_j1 *
            M_robotBase_j2 * M_rot_j2 *
            M_j2_j3 * M_rot_j3 *
            M_j3_j4 * M_rot_j4 *
            M_rot_j5 * M_rot_j6 * M_j6_eef;

    std::cout << " m_fk " << std::endl;
    std::cout << m_fk << std::endl;

    Eigen::MatrixXd rb_eef(4,4);
    rb_eef = M_rot_j1 *
            M_robotBase_j2 * M_rot_j2 *
            M_j2_j3 * M_rot_j3 *
            M_j3_j4 * M_rot_j4 *
            M_rot_j5 * M_rot_j6 * M_j6_eef;

    std::cout << " rb_eef " << std::endl;
    std::cout << rb_eef << std::endl;

    // intermediate matrices
    // world -> base
    Eigen::MatrixXd mW0(4,4), m01(4,4), m02(4,4), m03(4,4), m04(4,4), m05(4,4), m06(4,4), m0eef(4,4);
    mW0 = M_world_robotBase;
    // base -> j1
    m01 = eye4 * M_rot_j1;
    // base -> j2
    m02 = m01 * M_robotBase_j2 * M_rot_j2;
    // base -> j3
    m03 = m02 * M_j2_j3 * M_rot_j3;
    // base -> j4
    m04 = m03 * M_j3_j4 * M_rot_j4;
    // base -> j5
    m05 = m04 * M_rot_j5;
    // base -> j6
    m06 = m05 * M_rot_j6;
    // base -> eef
    m0eef = m06 * M_j6_eef;

    // -------------------------
    // end effector pose
    double x_eef = rb_eef(0,3);
    double y_eef = rb_eef(1,3);
    double z_eef = rb_eef(2,3);

    Eigen::MatrixXd rot(3,3);
    rot <<  rb_eef(0,0), rb_eef(0,1), rb_eef(0,2),
            rb_eef(1,0), rb_eef(1,1), rb_eef(1,2),
            rb_eef(2,0), rb_eef(2,1), rb_eef(2,2);

    Eigen::Vector3d zyx_angle;
    rotationMatrixToZYXeuler(rot, zyx_angle);

    double roll_eef = zyx_angle(2) *180/M_PI;
    double pitch_eef = zyx_angle(1) *180/M_PI;
    double yaw_eef = zyx_angle(0) *180/M_PI;
    std::cout << "roll : " << roll_eef <<
                 " , pitch : " << pitch_eef <<
                 " , yaw : " << yaw_eef << std::endl;

    // returns
    if(xyz_eef.size()!= 3)
        xyz_eef.resize(3);

    if(rot_eef.cols()!= 3 | rot_eef.rows()!= 3)
        rot_eef.resize(3,3);

    xyz_eef << x_eef , y_eef, z_eef;
    rot_eef = rot;

    std::cout << " xyz_eef " << std::endl;
    std::cout << xyz_eef << std::endl;
    std::cout << " rot_eef " << std::endl;
    std::cout << rot_eef << std::endl;

    return true;

}

Eigen::MatrixXd RobotState::computeBaseToSphWristPoint_old(double theta1, double theta2, double theta3){

    double theta3offset = -M_PI/4;
    theta3 = theta3 +theta3offset;

    // MATRICES

    Eigen::MatrixXd eye3(3,3); eye3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd eye4(4,4); eye4 = Eigen::Matrix4d::Identity();
    Eigen::Vector3d zeros_; zeros_<< 0, 0, 0;

    // ---- ROTATION J1
    Eigen::MatrixXd R_j1(3,3); rotationAroundZ(theta1, R_j1);
    Eigen::MatrixXd M_rot_j1(4,4);
    M_rot_j1 << R_j1, zeros_,
            zeros_.transpose(), 1;

    // ---- ROBOT_BASE -> J2
    Eigen::MatrixXd R_robotBase_j2(3,3);  R_robotBase_j2= eye3;
    Eigen::Vector3d T_robotBase_j2;  T_robotBase_j2 << 0, 0, Z_RBJ2;
    Eigen::MatrixXd M_robotBase_j2(4,4);
    M_robotBase_j2 << R_robotBase_j2, T_robotBase_j2,
            zeros_.transpose(), 1;

    // ---- ROTATION J2
    Eigen::MatrixXd R_j2(3,3); rotationAroundY(theta2, R_j2);
    Eigen::MatrixXd M_rot_j2(4,4);
    M_rot_j2 << R_j2, zeros_,
            zeros_.transpose(), 1;

    // ---- J2 -> J3
    Eigen::MatrixXd R_j2_j3(3,3); R_j2_j3 = eye3;
    Eigen::Vector3d T_j2_j3; T_j2_j3  << X_J2J3, 0, Z_J2J3;
    Eigen::MatrixXd M_j2_j3(4,4);
    M_j2_j3 << R_j2_j3, T_j2_j3,
                zeros_.transpose(), 1;

    std::cout << " M_j2_j3 " << std::endl;
    std::cout << M_j2_j3 << std::endl;

    // ---- ROTATION J3
    Eigen::MatrixXd R_j3(3,3); rotationAroundY(theta3 - theta3offset, R_j3);
    Eigen::MatrixXd M_rot_j3(4,4);
    M_rot_j3 << R_j3, zeros_,
        zeros_.transpose(), 1;

    std::cout << " M_rot_j3 " << std::endl;
    std::cout << M_rot_j3 << std::endl;

    // ---- J3 -> J4 (spherical wrist mid point)
    Eigen::MatrixXd R_j3_j4(3,3); R_j3_j4 = eye3;
    Eigen::Vector3d T_j3_j4; T_j3_j4 << X_J3J4, 0, Z_J3J4;
    Eigen::MatrixXd M_j3_j4(4,4);
    M_j3_j4 << R_j3_j4, T_j3_j4,
            zeros_.transpose(), 1;

    std::cout << " M_j3_j4 " << std::endl;
    std::cout << M_j3_j4 << std::endl;

    Eigen::MatrixXd m(4,4);
    m = M_rot_j1 *
            M_robotBase_j2 * M_rot_j2 *
            M_j2_j3 * M_rot_j3 *
            M_j3_j4;

    return m;

}

bool RobotState::InverseKinematics_old(Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles){

    double ROBOT_0_OFFSET = M_PI/4;
    double l2 = sqrt(X_J2J3*X_J2J3 + Z_J2J3*Z_J2J3);
    double l3 = sqrt(X_J3J4*X_J3J4 + Z_J3J4*Z_J3J4);
    double l6 = 0.645-X_J3J4-Z_J2J3;

    // ------- SPHERICAL WRIST POINT DETERMINATION
    Eigen::MatrixXd r06(3,3); r06 = rot;

    Eigen::MatrixXd t06(4,4);
    t06 <<  r06, pos,
            0, 0, 0, 1;

    double xc = t06(0,3) - (l6 * t06(0,0));
    double yc = t06(1,3) - (l6 * t06(1,0));
    double zc = t06(2,3) - (l6 * t06(2,0));
    std::cout << "xc" << xc << "yc" << yc << "zc" << zc << std::endl;

    double r = sqrt(xc*xc+yc*yc);
    double s = zc-Z_RBJ2;

    double A = l2;
    double B = l3;

    double C = sqrt(r*r + s*s);
    double beta = atan(0.030/0.258);
    double gamma = atan(0.030/0.264);
    double ab, ac, alpha;

    std::cout << " ----------------- " << std::endl;
    std::cout << " C : " << C << std::endl;
    std::cout << " A+B : " << A+B  << std::endl;
    std::cout << " ----------------- " << std::endl;

    if(C <= A + B)
    {

        // find theta3_linha : c² = a² + b² -2*a*b*cos(a^b)

        //double cos_ab = - (C^2 - A*A - B*B)/(2*A*B);
        ab = 2*atan2(1, -sqrt((2*A*B+C*C-A*A-B*B)/(2*A*B-C*C+A*A+B*B)))- M_PI;

        //double cos_ac = - (B*B - A*A - C*C)/(2*A*C);
        ac = 2*atan2(1, -sqrt((2*A*C+B*B-A*A-C*C)/(2*A*C-B*B+A*A+C*C)))- M_PI;

        alpha = M_PI/2-atan(0.030/0.264);

        // PC contas
        double theta1 = atan2(yc,xc);

        double theta2_linha = atan2(s,r) + ac; // S, R
        double theta3_linha = ab - M_PI/2;

        double theta2, theta3;
        if (roundToNdecimalPlaces(ac,4) == roundToNdecimalPlaces(gamma, 4))
        {
            theta2 = theta2_linha -gamma;
            theta3 = theta3_linha +gamma +beta;
        }
        else
        {
            theta2 = theta2_linha + gamma;
            theta3 = theta3_linha -gamma -beta;
        }

        // os angulos que daria supostamente ao robot
        double theta1_robot = theta1;
        double theta2_robot = M_PI/2 - theta2 ;
        double theta3_robot = -(theta3);

        // Encontrar theta4, theta5, theta6

        Eigen::MatrixXd m03(4,4);
        m03 = computeBaseToSphWristPoint_old(theta1_robot, theta2_robot, theta3_robot);

        Eigen::MatrixXd r03(3,3); r03 = m03.block(0,0,3,3);

        Eigen::MatrixXd r_matrix(3,3);
        r_matrix = (r03).transpose()*rot;

        double theta4, theta5, theta6;

        std::cout << roundToNdecimalPlaces(r_matrix(0,0),4) << std::endl;
        if(roundToNdecimalPlaces(r_matrix(0,0),4) == 1)
           theta5 = 0;
        else if(roundToNdecimalPlaces(r_matrix(0,0),4) == -1)
           theta5 = M_PI;
        else
           theta5 = -M_PI/2 - atan2(r_matrix(0,0), -sqrt(1-r_matrix(0,0)*r_matrix(0,0)));

        theta6 = -M_PI/2 - atan2(-r_matrix(0,2), -r_matrix(0,1)) ;
        theta4 = -M_PI/2 - atan2( r_matrix(2,0), -r_matrix(1,0)) ;

        // obs: the eq of theta4, theta5 and theta6 are specified to have the
        // manipulator in the way I want. Check slides
        theta4 = normalizeAngle(theta4);
        theta5 = normalizeAngle(theta5);
        theta6 = normalizeAngle(theta6);

        if (theta1_robot == M_PI) theta1_robot =0;
        if (roundToNdecimalPlaces(theta4, 4) == -roundToNdecimalPlaces(M_PI, 4)) theta4 =0;
        if (roundToNdecimalPlaces(theta4, 4) == roundToNdecimalPlaces(M_PI, 4)) theta4 =0;
        if (roundToNdecimalPlaces(theta6, 4) == -roundToNdecimalPlaces(M_PI, 4)) theta6 =0;
        if (roundToNdecimalPlaces(theta6, 4) == roundToNdecimalPlaces(M_PI, 4)) theta6 =0;

        std::cout << "theta4 : " << theta4 << std::endl;
        std::cout << "theta6 : " << theta6 << std::endl;

        // return joint angles
        joint_angles.resize(6);
        joint_angles(0) = roundToNdecimalPlaces(theta1_robot, 4);
        joint_angles(1) = roundToNdecimalPlaces(theta2_robot, 4);
        joint_angles(2) = roundToNdecimalPlaces(theta3_robot, 4) + ROBOT_0_OFFSET;
        joint_angles(3) = roundToNdecimalPlaces(theta4, 4);
        joint_angles(4) = roundToNdecimalPlaces(theta5, 4);
        joint_angles(5) = roundToNdecimalPlaces(theta6, 4);


        std::cout << "ab " << ab << std::endl;
        std::cout << "ac " << ac << std::endl;
        std::cout << "alpha " << alpha << std::endl;

        std::cout << "theta2_linha " << theta2_linha << std::endl;
        std::cout << "theta3_linha " << theta3_linha << std::endl;
        std::cout << "theta2 " << theta2 << std::endl;
        std::cout << "theta3 " << theta3 << std::endl;
        std::cout << "theta2_robot " << theta2_robot << std::endl;
        std::cout << "theta3_robot " << theta3_robot << std::endl;

        std::cout << "m03 " << std::endl <<m03 << std::endl;

        std::cout << "r_matrix " << std::endl <<r_matrix << std::endl;

        std::cout << "theta4 " << theta4 << std::endl;
        std::cout << "theta5 " << theta5 << std::endl;
        std::cout << "theta6 " << theta6 << std::endl;


        std::cout << "joint_angles " << std::endl <<joint_angles << std::endl;


        return true;
    }
    else
    {
        return false;
    }


}

