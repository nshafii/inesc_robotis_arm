#include <iostream>


#include <vector>
#include <iostream>
#include <Eigen/Dense>

// NOTA : verificar o offset!!!


void rotationAroundZ(double theta, Eigen::MatrixXd &rotation);
void rotationAroundX(double theta, Eigen::MatrixXd &rotation);
void rotationAroundY(double theta, Eigen::MatrixXd &rotation);
void rotationMatrixToZYXeuler(Eigen::MatrixXd rot, Eigen::Vector3d &zyx_angle);
double normalizeAngle(double t);
double roundToNdecimalPlaces( double x , int n);
Eigen::MatrixXd computeBaseToSphWristPoint(double theta1, double theta2, double theta3);

bool InverseKinematics(Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles){

    double l1 = 0.159 ;
    double l2 = sqrt(0.030*0.030 + 0.264*0.264);
    double l3 = sqrt(0.030*0.030 + 0.258*0.258);
    double l6 = 0.645-0.258-0.264;

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
    double s = zc-l1;

    double A = l2;
    double B = l3;

    double C = sqrt(r*r + s*s);
    double beta = atan(0.030/0.258);
    double gamma = atan(0.030/0.264);
    double ab, ac, alpha;

    if(C <= A + B)

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
        m03 = computeBaseToSphWristPoint(theta1_robot, theta2_robot, theta3_robot);

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
           theta5 = M_PI/2 - atan2(r_matrix(0,0), -sqrt(1-r_matrix(0,0)*r_matrix(0,0)));

        theta6 = M_PI/2 - atan2(-r_matrix(0,2), -r_matrix(0,1)) ;
        theta4 = M_PI/2 - atan2( r_matrix(2,0), -r_matrix(1,0)) ;

        // obs: the eq of theta4, theta5 and theta6 are specified to have the
        // manipulator in the way I want. Check slides
        theta4 = normalizeAngle(theta4);
        theta5 = normalizeAngle(theta5);
        theta6 = normalizeAngle(theta6);

        if (theta1_robot == M_PI) theta1_robot =0;
        if (theta4 == -M_PI) theta4 =0;
        if (theta6 == -M_PI) theta6 =0;


        // return joint angles
        joint_angles.resize(6);
        joint_angles(0) = roundToNdecimalPlaces(theta1_robot, 4);
        joint_angles(1) = roundToNdecimalPlaces(theta2_robot, 4);
        joint_angles(2) = roundToNdecimalPlaces(theta3_robot, 4);
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


}



double normalizeAngle(double t)
{
    double t_;

    t_ = fmod(t+M_PI, 2*M_PI) + -M_PI;

    return t_;
}

Eigen::MatrixXd computeBaseToSphWristPoint(double theta1, double theta2, double theta3){

    double theta3offset = -M_PI/4;
    theta3 = theta3 +theta3offset;

    // MEASUREMENTS
    double Z_RBJ2 = 0.159;

    double X_J2J3 = 0.030;
    double Z_J2J3 = 0.264;

    double X_J3J4 = 0.258;
    double Z_J3J4 = 0.030;

    double X_J6EEF = 0.645-0.258-0.264;

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


bool ForwardKinematics(Eigen::VectorXd joint_angles, Eigen::Vector3d &xyz_eef, Eigen::MatrixXd &rot_eef){

    double theta3offset = -M_PI/4;

    if(joint_angles.size() != 6)
    {
        std::cout << "Number of joint angles must be 6!" << std::endl;
        return false;
    }

    //set theta angles
    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2] +theta3offset;
    double theta4 = joint_angles[3];
    double theta5 = joint_angles[4];
    double theta6 = joint_angles[5];


    // MEASUREMENTS
    double x_w_rb = 0.009+0.009/2.0;
    double y_w_rb = 0.009+0.009/2.0;
    double z_w_rb = 0.006+0.017;

    double Z_RBJ2 = 0.159;

    double X_J2J3 = 0.030;
    double Z_J2J3 = 0.264;

    double X_J3J4 = 0.258;
    double Z_J3J4 = 0.030;

    double X_J6EEF = 0.645-0.258-0.264;

    // MATRICES

    Eigen::MatrixXd eye3(3,3); eye3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd eye4(4,4); eye4 = Eigen::Matrix4d::Identity();
    Eigen::Vector3d zeros_; zeros_<< 0, 0, 0;

    // ---- WORLD -> ROBOT_BASE
    Eigen::MatrixXd R_world_robotBase(3,3); R_world_robotBase = eye3;
    Eigen::Vector3d T_world_robotBase; T_world_robotBase << x_w_rb, y_w_rb, z_w_rb;
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


void rotationAroundZ(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0,          0,          1;

}



void rotationAroundX(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << 1.0, 	 0.0, 	  0.0,
            0.0, cos(theta), -sin(theta),
            0.0, sin(theta),  cos(theta);

}


void rotationAroundY(double theta, Eigen::MatrixXd &rot){

    if(rot.cols() != 3 | rot.rows() != 3)
        rot.resize(3,3);

    rot << cos(theta), 0.0, sin(theta),
            0.0, 	 1.0, 	 0.0,
            -sin(theta), 0.0, cos(theta);
}


void rotationMatrixToZYXeuler(Eigen::MatrixXd rot, Eigen::Vector3d &zyx_angle)
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



double roundToNdecimalPlaces( double x , int n)
{
    const double sd = pow(10, n);
    double a  =int(x*sd + (x<0? -0.5 : 0.5))/sd;
    return a;
}


int main(){

    Eigen::VectorXd joint_angles(6);
    joint_angles << -M_PI/3, M_PI/4, -M_PI/2, M_PI/3, M_PI/2, M_PI/3;

    Eigen::Vector3d xyz_eef(3);
    Eigen::MatrixXd rot_eef(3,3);
//    ForwardKinematics(joint_angles, xyz_eef, rot_eef);

    xyz_eef << 0.4188, 0, 0.2979;
    rot_eef << 0.9211, 0, 0.3894,
                0, 1, 0,
                -0.3894, 0, 0.9211;

    InverseKinematics(xyz_eef, rot_eef, joint_angles);


    return 0;
}
