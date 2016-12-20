#include "../../INESC_Robotis_Driver/include/INESC_Robotis_Driver/IK_Solver.hpp"

#include <iostream>

/*****************************************************************************
** Implementation [Robot_state]
*****************************************************************************/

IK_Solver::IK_Solver()
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

sensor_msgs::JointState IK_Solver::getRobotJointStates()
{
    return joint_states_;
}

void IK_Solver::setRobotJointStates(const sensor_msgs::JointState::ConstPtr& current_joint_states)
{
 joint_states_.header = current_joint_states->header;
 joint_states_.position = current_joint_states->position;
 joint_states_.name = current_joint_states->name;
}

sensor_msgs::JointState IK_Solver::getGoalJointStates()
{
    return goal_joint_states_;
}

void IK_Solver::setGoalJointStates(const sensor_msgs::JointState::ConstPtr& goal_joint_states)
{
 goal_joint_states_.header = goal_joint_states->header;
 goal_joint_states_.position = goal_joint_states->position;
 goal_joint_states_.name = goal_joint_states->name;
}

void IK_Solver::setGoalJointStates(std::vector<double> joint_values)
{
    goal_joint_states_.header.stamp = ros::Time::now();

    goal_joint_states_.position[0] = joint_values[0];
    goal_joint_states_.position[1] = joint_values[1];
    goal_joint_states_.position[2] = joint_values[2];
    goal_joint_states_.position[3] = joint_values[3];
    goal_joint_states_.position[4] = joint_values[4];
    goal_joint_states_.position[5] = joint_values[5];
}

std::vector<double> IK_Solver::getHomePosition(){
    return home_position_joints_;
}

bool IK_Solver::checkRobotPos(std::vector<double> goal_joints){

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

bool IK_Solver::checkGoalJointViability()
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

bool IK_Solver::checkGoalJointViabilityAndCorrect()
{
    bool viability = true;
    if( goal_joint_states_.position[0] < JOINT1_LOWER_LIMIT)
    {
      goal_joint_states_.position[0] = JOINT1_LOWER_LIMIT;
      viability = false;
    }

    if(goal_joint_states_.position[0] > JOINT1_UPPER_LIMIT)
    {
      goal_joint_states_.position[0] = JOINT1_UPPER_LIMIT;
      viability = false;
    }

    if( goal_joint_states_.position[1] < JOINT2_LOWER_LIMIT)
    {
      goal_joint_states_.position[1] = JOINT2_LOWER_LIMIT;
      viability = false;
    }

    if(goal_joint_states_.position[1] > JOINT2_UPPER_LIMIT){
      goal_joint_states_.position[1] = JOINT2_UPPER_LIMIT;
      viability = false;
    }

    if( goal_joint_states_.position[2] < JOINT3_LOWER_LIMIT){
      goal_joint_states_.position[2] = JOINT3_LOWER_LIMIT;
      viability = false;
    }

    if(goal_joint_states_.position[2] > JOINT3_UPPER_LIMIT){
      goal_joint_states_.position[2] = JOINT3_UPPER_LIMIT;
      viability = false;
    }

    if( goal_joint_states_.position[3] < JOINT4_LOWER_LIMIT){
      goal_joint_states_.position[3] = JOINT4_LOWER_LIMIT;
      viability = false;
    }

    if( goal_joint_states_.position[3] > JOINT4_UPPER_LIMIT){
      goal_joint_states_.position[3] = JOINT4_UPPER_LIMIT;
      viability =  false;
    }

    if( goal_joint_states_.position[4] < JOINT5_LOWER_LIMIT){
      goal_joint_states_.position[4] = JOINT5_LOWER_LIMIT;
      viability =  false;
    }

    if( goal_joint_states_.position[4] > JOINT5_UPPER_LIMIT){
      goal_joint_states_.position[4] = JOINT5_LOWER_LIMIT;
      viability =  false;
    }

    if( goal_joint_states_.position[5] < JOINT6_LOWER_LIMIT){
      goal_joint_states_.position[5] = JOINT6_LOWER_LIMIT;
      viability =  false;
    }

    if( goal_joint_states_.position[5] > JOINT6_UPPER_LIMIT){
      goal_joint_states_.position[5] = JOINT6_LOWER_LIMIT;
      viability =  false;
    }

    return viability;
}

void IK_Solver::vectorToPoseStamped(std::vector<double> pos, geometry_msgs::PoseStamped& pose){
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

void IK_Solver::quaternionToRotation(Eigen::Matrix3d &R, double q1, double q2, double q3, double q0){

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

Eigen::MatrixXd IK_Solver::rpy2rotation( double r, double p, double y )
{
    Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

    return R;
}

Eigen::Quaterniond IK_Solver::rpy2quaternion( double r, double p, double y )
{
    Eigen::Matrix3d R = rpy2rotation(r, p, y);

    Eigen::Matrix3d R_plus;
    R_plus = R.block(0,0,3,3);

    Eigen::Quaterniond QR;
    QR = R_plus;

    return QR;
}

void IK_Solver::rotationAroundZ(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0,          0,          1;

}

void IK_Solver::rotationAroundX(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << 1.0, 	 0.0, 	  0.0,
            0.0, cos(theta), -sin(theta),
            0.0, sin(theta),  cos(theta);

}

void IK_Solver::rotationAroundY(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << cos(theta), 0.0, sin(theta),
            0.0, 	 1.0, 	 0.0,
            -sin(theta), 0.0, cos(theta);
}

void IK_Solver::rotationMatrixToZYXeuler(Eigen::MatrixXd rot, Eigen::Vector3d &zyx_angle)
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



Eigen::MatrixXd IK_Solver::rotationMatrixFromZYXeuler(std::vector<double> zyx_eulerAngles)
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

Eigen::MatrixXd IK_Solver::rotationMatrixFromUnitQuaternion(std::vector<double> q)
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

std::vector<double> IK_Solver::rotationMatrixToUnitQuaternion(Eigen::MatrixXd r)
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

double IK_Solver::roundToNdecimalPlaces( double x , int n)
{
    const double sd = pow(10, n);
    double a  = int(x*sd + (x<0? -0.5 : 0.5))/sd;
    return a;
}

double IK_Solver::normalizeAngle(double t)
{
    double t_;

    t_ = fmod(t+M_PI, 2*M_PI) + -M_PI;

    return t_;
}

double IK_Solver::my_normAngle(double alpha)
{
    // btw -pi and pi
    double angle_ = atan2(sin(alpha), cos(alpha));
    return angle_;
}

// -------------------------------------------------------------------------------------------------------------
//                                              KINEMATICS
// -------------------------------------------------------------------------------------------------------------

bool IK_Solver::ForwardKinematics(Eigen::VectorXd joint_angles, Eigen::Vector3d &xyz_eef, Eigen::MatrixXd &rot_eef){

    double A1 = 0.159;
    double A2 = sqrt(0.264*0.264+0.030*0.030);
    double OFFSET_JOINT2 = (M_PI/2-atan2(0.030,0.264));
    //double OFFSET_JOINT3  = (M_PI/4 +atan2(0.030,0.264)); // cat code
    double OFFSET_JOINT3  = (atan2(0.030,0.264)); // new
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

void IK_Solver::computeBaseToSphWristPoint(double theta1, double theta2, double theta3, Eigen::Vector3d &pos, Eigen::MatrixXd &rot){

    double A1 = 0.159;
    double A2 = sqrt(0.264*0.264+0.030*0.030);
    double OFFSET_JOINT2 = (M_PI/2-atan2(0.030,0.264));
    //double OFFSET_JOINT3  = (M_PI/4 +atan2(0.030,0.264));
    double OFFSET_JOINT3  = (atan2(0.030,0.264)); //new
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

bool IK_Solver::InverseKinematics(Eigen::VectorXd current_joint_angles, Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles){

    double l1 = 0.159 ;
    double l2 = sqrt(X_J2J3*X_J2J3 + Z_J2J3*Z_J2J3);
    double l3 = sqrt(X_J3J4*X_J3J4 + Z_J3J4*Z_J3J4);
    double l6 = 0.645-X_J3J4-Z_J2J3;
    double beta = atan(0.030/0.258);
    double gamma = atan(0.030/0.264);

    // ------- Construct Transformation matrix from 0 to 6
    Eigen::MatrixXd R06(3,3);

    Eigen::MatrixXd correctionRotation(3,3);
    correctionRotation<< cos(0),-sin(0),0, sin(0), cos(0),0, 0,0,1;

    R06 = rot*correctionRotation;

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
        double theta4_2, theta5_2, theta6_2;

        double theta4_robot, theta5_robot, theta6_robot;
        double theta4_2_robot, theta5_2_robot, theta6_2_robot;


        Eigen::MatrixXd R03(3,3);
        Eigen::Vector3d t03;
        //computeBaseToSphWristPoint(theta1_robot, theta2_robot, theta3_robot+M_PI/4, t03, R03);
        computeBaseToSphWristPoint(theta1_robot, theta2_robot, theta3_robot, t03, R03);// new

        Eigen::MatrixXd r_matrix(3,3);
        r_matrix = (R03).transpose()*R06;

        for (int i=0; i< r_matrix.rows(); i++)
            for(int j=0; j < r_matrix.cols(); j++)
                r_matrix(i,j) = roundToNdecimalPlaces(r_matrix(i,j),4);

        double r11 = r_matrix(0,0); double r13 = r_matrix(0,2); double r31 = r_matrix(2,0);
        double r23 = r_matrix(1,2); double r32 = r_matrix(2,1); double r33 = r_matrix(2,2);
        double r21 = r_matrix(2,1);
        if(roundToNdecimalPlaces(r11,5) == 1)
        {
            // what is this else
          std::cout<<"OPS R11 = 1 !!!!!!!!!"<<std::endl;
//          theta5 = 0;
//          theta4 = current_joint_angles(3);
//          theta6 = atan2(r11 , r21) - theta4;
//          theta5_robot =  theta5;
//          theta4_robot =  theta4;
//          theta6_robot =   my_normAngle(theta6);
//          joint_angles << theta1_robot, theta2_robot, theta3_robot,
//                        theta4_robot, theta5_robot, theta6_robot;

          std::cout<< "matrix R3_6: "<< r_matrix <<std::endl;
          theta4 = atan2(-r13, -r23);
          std::cout<< "theta 4: "<< theta4 <<std::endl;

          theta5 = atan2( r33, -sqrt(1-r33*r33));
          std::cout<< "theta 5: "<< theta5 <<std::endl;

          theta6 = atan2( r31, -r32);
          std::cout<< "theta 6: "<< theta6 <<std::endl;

          theta4_2 = atan2( r13, r23);
          std::cout<< "theta 4_2: "<< theta4_2 <<std::endl;

          theta5_2 = atan2( r33, sqrt(1-r33*r33));
          std::cout<< "theta 5_2: "<< theta5_2 <<std::endl;

          theta6_2 = atan2(-r31, r32);
          std::cout<< "theta 6_2: "<< theta6_2 <<std::endl;

          theta5_2_robot = -M_PI/2 - theta5_2;
          theta4_2_robot =  M_PI/2 - theta4_2;
          theta6_2_robot =  M_PI/2 - theta6_2;

          theta5_robot = -M_PI/2 - theta5;
          theta4_robot =  M_PI/2 - theta4;
          theta6_robot =  M_PI/2 - theta6;

          theta6_robot = my_normAngle(theta6_robot);
          std::cout<< "theta6_robot: "<< theta6_robot <<std::endl;

          std::cout<< "theta5_robot befor: "<< theta5_robot <<std::endl;

          theta5_robot = M_PI - my_normAngle(theta5_robot);
          theta5_robot = my_normAngle(theta5_robot);
          theta5_robot = my_normAngle(theta5_robot);

          std::cout<< "theta5_robot: "<< theta5_robot <<std::endl;

          theta4_robot = my_normAngle(theta4_robot);
          std::cout<< "theta4_robot: "<< theta4_robot <<std::endl;

          theta6_2_robot = my_normAngle(theta6_2_robot);
          double theta5_2_robot_temp = M_PI - my_normAngle(theta5_2_robot);
          theta5_2_robot = my_normAngle(theta5_2_robot_temp);
          theta4_2_robot = my_normAngle(theta4_2_robot);

          double rotationFirstSolution =
              (2*shortestAngleDiff(theta4_robot,current_joint_angles(3)))+
              (2*shortestAngleDiff(theta5_robot,current_joint_angles(4)))+
              shortestAngleDiff(theta6_robot,current_joint_angles(5));
          std::cout<< "rotationFirstSolution: "<< rotationFirstSolution <<std::endl;

          double rotationSecondSolution =
              (2*shortestAngleDiff(theta4_2_robot,current_joint_angles(3)))+
              (2*shortestAngleDiff(theta5_2_robot,current_joint_angles(4)))+
              shortestAngleDiff(theta6_2_robot,current_joint_angles(5));
          std::cout<< "rotationSecondSolution: "<< rotationSecondSolution <<std::endl;


          if (rotationFirstSolution < rotationSecondSolution){
            std::cout << "Fisrt solution, Total rotation first:"
                << rotationFirstSolution << "and second:" << rotationSecondSolution << std::endl;
            joint_angles << theta1_robot, theta2_robot, theta3_robot,
                          theta4_robot, theta5_robot, theta6_robot;
          }
          else{
            std::cout << " !!!!Second solution, Total rotation first:"
                << rotationFirstSolution << "and second:" << rotationSecondSolution <<std::endl;

            joint_angles << theta1_robot, theta2_robot, theta3_robot,
                          theta4_2_robot, theta5_2_robot, theta6_2_robot;

          }

        }
        else if(roundToNdecimalPlaces(r11,5) == -1)
        {
            // what is this el
          std::cout<<"OPS R11 = -1 !!!!!!!!!"<<std::endl;
//
//          theta5 = 0;
//          theta4 = current_joint_angles(3);
//          theta6 = theta4 - atan2(-r11 , r21);
//          theta5_robot =  theta5;
//          theta4_robot =  theta4;
//          theta6_robot =  my_normAngle(theta6);
//          joint_angles << theta1_robot, theta2_robot, theta3_robot,
//                        theta4_robot, theta5_robot, theta6_robot;
//
          std::cout<<"OPS R11 = -1 !!!!!!!!!"<<std::endl;


          theta4 = atan2(-r13, -r23);
          theta5 = atan2( r33, -sqrt(1-r33*r33));
          theta6 = atan2( r31, -r32);

          theta4_2 = atan2( r13, r23);
          theta5_2 = atan2( r33, sqrt(1-r33*r33));
          theta6_2 = atan2(-r31, r32);

          theta5_2_robot = -M_PI/2 - theta5_2;
          theta4_2_robot =  M_PI/2 - theta4_2;
          theta6_2_robot =  M_PI/2 - theta6_2;

          theta5_robot = -M_PI/2 - theta5;
          theta4_robot =  M_PI/2 - theta4;
          theta6_robot =  M_PI/2 - theta6;

          theta6_robot = my_normAngle(theta6_robot);
          theta5_robot = M_PI - my_normAngle(theta5_robot);
          theta4_robot = my_normAngle(theta4_robot);

          theta6_2_robot = my_normAngle(theta6_2_robot);
          double theta5_2_robot_temp = M_PI - my_normAngle(theta5_2_robot);
          theta5_2_robot = my_normAngle(theta5_2_robot_temp);
          theta4_2_robot = my_normAngle(theta4_2_robot);

          double rotationFirstSolution =
              (2*shortestAngleDiff(theta4_robot,current_joint_angles(3)))+
              (2*shortestAngleDiff(theta5_robot,current_joint_angles(4)))+
              shortestAngleDiff(theta6_robot,current_joint_angles(5));

          double rotationSecondSolution =
              (2*shortestAngleDiff(theta4_2_robot,current_joint_angles(3)))+
              (2*shortestAngleDiff(theta5_2_robot,current_joint_angles(4)))+
              shortestAngleDiff(theta6_2_robot,current_joint_angles(5));

          if (rotationFirstSolution < rotationSecondSolution){
            std::cout << "Fisrt solution, Total rotation first:"
                << rotationFirstSolution << "and second:" << rotationSecondSolution << std::endl;
            joint_angles << theta1_robot, theta2_robot, theta3_robot,
                          theta4_robot, theta5_robot, theta6_robot;
          }
          else{
            std::cout << " !!!!Second solution, Total rotation first:"
                << rotationFirstSolution << "and second:" << rotationSecondSolution <<std::endl;

            joint_angles << theta1_robot, theta2_robot, theta3_robot,
                          theta4_2_robot, theta5_2_robot, theta6_2_robot;

          }

        }
        else
        {
            theta4 = atan2(-r13, -r23);
            theta5 = atan2( r33, -sqrt(1-r33*r33));
            theta6 = atan2( r31, -r32);

            theta4_2 = atan2( r13, r23);
            theta5_2 = atan2( r33, sqrt(1-r33*r33));
            theta6_2 = atan2(-r31, r32);

            theta5_2_robot = -M_PI/2 - theta5_2;
            theta4_2_robot =  M_PI/2 - theta4_2;
            theta6_2_robot =  M_PI/2 - theta6_2;

            theta5_robot = -M_PI/2 - theta5;
            theta4_robot =  M_PI/2 - theta4;
            theta6_robot =  M_PI/2 - theta6;

            theta6_robot = my_normAngle(theta6_robot);
            theta5_robot = M_PI - my_normAngle(theta5_robot);
            theta5_robot = my_normAngle(theta5_robot);
            theta4_robot = my_normAngle(theta4_robot);

            theta6_2_robot = my_normAngle(theta6_2_robot);
            double theta5_2_robot_temp = M_PI - my_normAngle(theta5_2_robot);
            theta5_2_robot = my_normAngle(theta5_2_robot_temp);
            theta4_2_robot = my_normAngle(theta4_2_robot);

            double rotationFirstSolution =
                (2*shortestAngleDiff(theta4_robot,current_joint_angles(3)))+
                (2*shortestAngleDiff(theta5_robot,current_joint_angles(4)))+
                shortestAngleDiff(theta6_robot,current_joint_angles(5));

            double rotationSecondSolution =
                (2*shortestAngleDiff(theta4_2_robot,current_joint_angles(3)))+
                (2*shortestAngleDiff(theta5_2_robot,current_joint_angles(4)))+
                shortestAngleDiff(theta6_2_robot,current_joint_angles(5));

            if (rotationFirstSolution < rotationSecondSolution){
              std::cout << "Fisrt solution, Total rotation first:"
                  << rotationFirstSolution << "and second:" << rotationSecondSolution << std::endl;
              joint_angles << theta1_robot, theta2_robot, theta3_robot,
                            theta4_robot, theta5_robot, theta6_robot;
            }
            else{
              std::cout << " !!!!Second solution, Total rotation first:"
                  << rotationFirstSolution << "and second:" << rotationSecondSolution <<std::endl;

              joint_angles << theta1_robot, theta2_robot, theta3_robot,
                            theta4_2_robot, theta5_2_robot, theta6_2_robot;

            }

        }

        goal_joint_states_.position[0]=joint_angles(0);
        goal_joint_states_.position[1]=joint_angles(1);
        goal_joint_states_.position[2]=joint_angles(2);
        goal_joint_states_.position[3]=joint_angles(3);
        goal_joint_states_.position[4]=joint_angles(4);
        goal_joint_states_.position[5]=joint_angles(5);

        return true;
    }
    else
    {
        std::cout << " XYZ given is not possible to reach." << std::endl;
        return false;
    }
}

double IK_Solver::shortestAngleDiff(double target, double source){
 std::cout<<"shortestAngleDiff:" << std::endl;
 double diff= fabs(atan2(sin(target-source),cos(target-source)));

 if( target*source < 0 and fabs(source) > M_PI/2)
 {
   std::cout<< "strange movement from dynamicel, punish the movement" << std::endl;
   diff= 2*M_PI - diff;
 }

 std::cout<< diff <<std::endl;

 return diff;
}

Eigen::MatrixXd IK_Solver::dh_transformation(Eigen::Vector4d dh_parameters){
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

void IK_Solver::findSingularities(Eigen::VectorXd joint_angles, Eigen::MatrixXd &J, bool &is_sing_J11, bool &is_sing_J22,
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


Eigen::MatrixXd IK_Solver::computeBaseToSphWristPoint_old(double theta1, double theta2, double theta3){

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

