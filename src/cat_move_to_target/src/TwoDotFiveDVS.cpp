#include "../include/cat_move_to_target/TwoDotFiveDVS.hpp"


//TwoDotFiveDVS::TwoDotFiveDVS(){
//    joint_state_.resize(6);
//    joint_angles_to_send.resize(6);
//    joint_angles_received.resize(6); // this is to be sure nthing changes during the code execution

//    is_set_s_xyz            =   false;
//    is_set_sStar_xyz        =   false;
//    is_set_theta_u          =   false;
//    is_set_L                =   false;
//    is_set_error            =   false;
//    is_set_lambda           =   false;
//    is_set_M                =   false;
//    is_set_cam_v            =   false;
//    is_set_eef_v            =   false;
//    is_set_J                =   false;
//    is_set_Ji               =   false;
//    is_set_R_ec             =   false;
//    is_set_R_ec_star        =   false;


//    //TODO check what to do with robot state

//    std::cout << " ! L ! " << std::endl;
//    for (int i=0; i<L.rows(); i++)
//    {
//        for(int j =0; j< L.cols(); j++)
//            std::cout << L.coeff(i,j) << " " ;
//        std::cout << std::endl;
//    }

//    std::cout << " ! error ! " << std::endl;
//    for (int i=0; i<L.rows(); i++)
//    {
//        for(int j =0; j< L.cols(); j++)
//            std::cout << L.coeff(i,j) << " " ;
//        std::cout << std::endl;
//    }
//}


//void TwoDotFiveDVS::set_p_xyz(std::vector<double> xyz)
//{
//    if(p_xyz.rows()!=3)
//         p_xyz.resize(3);
//    this->p_xyz(0)= xyz[0];
//    this->p_xyz(1) = xyz[1];
//    this->p_xyz(2) = xyz[2];

//    set_s_xyz();
//}

//void TwoDotFiveDVS::set_s_xyz()
//{
//    if(s_xyz.rows()!=3)
//        s_xyz.resize(3);
//    double X = this->p_xyz(0);
//    double Y = this->p_xyz(1);
//    double Z = this->p_xyz(2);
//    if(Z!=0)
//    {
//        this->s_xyz(0) = X/Z;
//        this->s_xyz(1) = Y/Z;
//        this->s_xyz(2) = log(Z);
//    }
//    else
//    {
//        std::cout << ("Z is zero") << std::endl;
//        this->s_xyz(0) = X/(Z+epsilon);
//        this->s_xyz(1) = Y/(Z+epsilon);
//        this->s_xyz(2) = log(Z+epsilon);
//    }

//    std::cout << " ! s ! " << std::endl;
//    for (int i=0; i<this->s_xyz.rows(); i++)
//        std::cout << this->s_xyz.coeff(i) << " " ;
//    std::cout << std::endl;

//    is_set_s_xyz = true;
//}

//void TwoDotFiveDVS::set_pStar_xyz(std::vector<double> xyz)
//{
//    if(pStar_xyz.rows()!=3)
//        pStar_xyz.resize(3);
//    this->pStar_xyz(0) = xyz[0];
//    this->pStar_xyz(1) = xyz[1];
//    this->pStar_xyz(2) = xyz[2];

//    set_sStar_xyz();
//}

//void TwoDotFiveDVS::set_sStar_xyz()
//{
//    if(sStar_xyz.rows()!=3)
//        sStar_xyz.resize(3);

//    double X = this->pStar_xyz(0);
//    double Y = this->pStar_xyz(1);
//    double Z = this->pStar_xyz(2);
//    if(Z!=0)
//    {
//        this->sStar_xyz(0) = X/Z;
//        this->sStar_xyz(1) = Y/Z;
//        this->sStar_xyz(2) = log(Z);
//    }
//    else
//    {
//        std::cout << ("Z is zero") << std::endl;
//        this->sStar_xyz(0) = X/(Z+epsilon);
//        this->sStar_xyz(1) = Y/(Z+epsilon);
//        this->sStar_xyz(2) = log(Z+epsilon);
//    }

//    std::cout << " ! s_star ! " << std::endl;
//    for (int i=0; i<this->sStar_xyz.rows(); i++)
//        std::cout << this->sStar_xyz.coeff(i) << " " ;
//    std::cout << std::endl;

//    is_set_sStar_xyz = true;
//}

//// ------------------------------------------------------------------------------
////                                LINEAR VELOCITY only
//// ------------------------------------------------------------------------------

//bool TwoDotFiveDVS::set_error_vl(){

//    if(error.rows()!=3)
//        error.resize(3);

//    if(is_set_s_xyz & is_set_sStar_xyz)
//    {

//        this->error(0) = this->s_xyz(0) - this->sStar_xyz(0);
//        this->error(1) = this->s_xyz(1) - this->sStar_xyz(1);
//        this->error(2) = log(this->p_xyz(2)/this->pStar_xyz(2));
//        this->is_set_error = true;
//        //---------------------------------
//        // print
//        std::cout << " ********** ERROER ****************** " << std::endl;
//        std::cout << " error " << std::endl << error << std::endl;
//        //---------------------------------

//    }
//    else
//    {
//        std::cout << "Couldn't set error. Parameters required not set" << std::endl;
//        return false;
//    }
//}

//void TwoDotFiveDVS::setInteractionMatrix_vl(){

//    if(L.cols()!= 3 & L.rows()!=3)
//    {
//        L.resize(3,3);
//        Lv.resize(3,3);
//    }

//    //TOCHECK please
//    //version with difference btw s and s* : not sure about this

//    std::cout << " ********** In setInteractionMatrix ****************** " << std::endl;

//    double x = this->error(0);
//    double y = this->error(1);
//    double Z = this->p_xyz(2); //TOCHECK please

//    // prints
//    std::cout << "x : " << x << " , "
//              << "y : " << y << " , "
//              << "Z : " << Z << std::endl;

//    //compute Lv
//    this->Lv <<   -1      ,   0   ,   x  ,
//            0      ,   -1  ,    y  ,
//            0      ,   0   ,   -1;

//    //compute L matrix
//    this->L << 1/Z*this->Lv;

//    std::cout << "Lv: " << std::endl << Lv << std::endl;

//    std::cout << "L: \n"<< L << std::endl;

//    is_set_L = true;

//}

//bool TwoDotFiveDVS::setCameraVelocity_vl(){

//    if(cam_v.rows()!=3)
//        cam_v.resize(3);

//    //TODO
//    if (is_set_lambda & is_set_L  & is_set_error)
//    {

//       Eigen::MatrixXd L_inverse(3,3);

//       Eigen::Matrix3d l = (this->Lv).inverse();

//       L_inverse = (this->Lv).inverse();

//       std::cout << "Here" << std::endl;
//       this->cam_v = - this->lambda * L_inverse * this->error;
//       //---------------------------------
//       // print
//       std::cout << " ********** In setCameraVelocity ****************** " << std::endl;
//       std::cout << " L inverse " << std::endl << L_inverse << std::endl;
//       std::cout << " cam v " << std::endl << cam_v << std::endl;
//       //---------------------------------

//        is_set_cam_v = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set camera velocity. Parameters required not set" << std::endl;
//        return false;
//    }


//}

//bool TwoDotFiveDVS::setEEFvelFromCameraVelocities_vl(){
//    if(eef_v.rows()!=3)
//        eef_v.resize(3);
//    if(is_set_cam_v & is_set_M)
//    {
//        this->eef_v << M * cam_v;

//        //---------------------------------
//        // print
//        std::cout << " ********** In setEEFvelFromCameraVelocities ****************** " << std::endl;
//        std::cout << " eef_v " << std::endl << eef_v << std::endl;
//        //---------------------------------

//        is_set_eef_v = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set eef velocity. Parameters required not set" << std::endl;
//        return false;
//    }

//}

//void TwoDotFiveDVS::setJacobian_vl(){

//    //TODO

//    // REVER ISTO COM NOVA FK
//    if(J.rows()!= 3 & J.cols()!=3)
//    {
//        J.resize(3,3);
//        Ji.resize(3,3);
//    }

//    //TODO check this, print, be thorough!!!
//    for (int i =0; i< joint_state_.size(); i++)
//        std::cout << "joint_state " << i << " : "  << joint_state_[i] <<std::endl;
//    for (int i =0; i< joint_state_.size(); i++)
//        joint_angles_received[i] = joint_state_[i];
//    // --------------------------------------------
//    double theta1 = joint_angles_received[0];
//    double theta2 = joint_angles_received[1];
//    double theta3 = joint_angles_received[2];
//    double theta3offset = -M_PI/4;

//    // -------------------------
//    // FORWARD KIN MATRIX

//    double c1 = cos(theta1); double  s1 = sin(theta1);
//    double c2 = cos(theta2); double s2 = sin(theta2);
//    double c3 = cos(theta3 - theta3offset);
//    double s3 = sin(theta3 - theta3offset);

//    // intermediate matrices
//    // base -> j1
//    Eigen::MatrixXd my_m01(4,4), my_m02(4,4), my_m03(4,4), my_m04(4,4);
//    my_m01 << c1, -s1, 0, 0 ,
//        s1, c1, 0, 0 ,
//         0,  0, 1, 0 ,
//         0,  0, 0, 1;
//    // base -> j2
//    my_m02 << c1*c2, -s1, c1*s2, 0,
//              s1*c2,  c1, s1*s2, 0,
//              -s2,     0,    c2, Z_RBJ2,
//                0,     0,     0, 1;
//    // base -> j3
//    my_m03 <<  c1*c2*c3-c1*s2*s3,  -s1,    c1*c2*s3+c1*s2*c3,   c1*c2*X_J2J3+c1*s2*Z_J2J3,
//               s1*c2*c3-s1*s2*s3,   c1,    s1*c2*s3+s1*s2*c3,   s1*c2*X_J2J3+s1*s2*Z_J2J3,
//                -s2*c3-c2*s3,        0,         -s2*s3+c2*c3,   -s2*X_J2J3+c2*Z_J2J3+Z_RBJ2,
//                0,                   0,                    0,                    1;

//    double A =  c1*c2*c3-c1*s2*s3;
//    double dA_d1 = -s1*c2*c3+s1*s2*s3;
//    double dA_d2 = -c1*s2*c3-c1*c2*s3;
//    double dA_d3 = -c1*c2*s3-c1*s2*c3;

//    double B = c1*c2*s3+c1*s2*c3 ;
//    double dB_d1 = -s1*c2*s3-s1*s2*c3 ;
//    double dB_d2 = -c1*s2*s3+c1*c2*c3 ;
//    double dB_d3 = c1*c2*c3-c1*s2*s3 ;

//    double C = c1*c2*X_J2J3+c1*s2*Z_J2J3;
//    double dC_d1 = -s1*c2*X_J2J3-s1*s2*Z_J2J3;
//    double dC_d2 = -c1*s2*X_J2J3+c1*c2*Z_J2J3;
//    double dC_d3 = 0;

//    double D  = s1*c2*c3-s1*s2*s3 ;
//    double dD_d1 = c1*c2*c3-c1*s2*s3 ;
//    double dD_d2 = -s1*s2*c3-s1*c2*s3 ;
//    double dD_d3 = -s1*c2*s3-s1*s2*c3 ;

//    double E = s1*c2*s3+s1*s2*c3 ;
//    double dE_d1 = c1*c2*s3+c1*s2*c3 ;
//    double dE_d2 = -s1*s2*s3+s1*c2*c3 ;
//    double dE_d3 = s1*c2*c3-s1*s2*s3 ;

//    double F = s1*c2*X_J2J3+s1*s2*Z_J2J3;
//    double dF_d1 = c1*c2*X_J2J3+c1*s2*Z_J2J3;
//    double dF_d2 = -s1*s2*X_J2J3+s1*c2*Z_J2J3;
//    double dF_d3 = 0;

//    double G = -s2*c3-c2*s3 ;
//    double dG_d1 = 0;
//    double dG_d2 = -c2*c3+s2*s3 ;
//    double dG_d3 = s2*s3-c2*c3 ;

//    double H = -s2*s3+c2*c3 ;
//    double dH_d1 = 0;
//    double dH_d2 = -c2*s3-s2*c3 ;
//    double dH_d3 = -s2*c3-c2*s3 ;

//    double I = -s2*X_J2J3+c2*Z_J2J3+Z_RBJ2;
//    double dI_d1 = 0;
//    double dI_d2 = c2*X_J2J3-s2*Z_J2J3;
//    double dI_d3 = 0;

//    // base -> spherical_wrist point
//    my_m04 <<   A ,   -s1 ,     B  ,   A*X_J3J4+B*Z_J3J4+C,
//                D ,    c1 ,     E  ,   D*X_J3J4+E*Z_J3J4+F,
//                G ,    0  ,     H  ,   G*X_J3J4+H*Z_J3J4+I,
//                0 ,    0  ,     0  ,   1;

//    Eigen::Vector3d dX_d1(3), dX_d2(3), dX_d3(3);
//    dX_d1 << dA_d1*X_J3J4 + dB_d1*Z_J3J4 + dC_d1,
//        dD_d1*X_J3J4 + dE_d1*Z_J3J4 + dF_d1,
//        dG_d1*X_J3J4 + dH_d1*Z_J3J4 + dI_d1;
//    dX_d2 << dA_d2*X_J3J4 + dB_d2*Z_J3J4 + dC_d2 ,
//        dD_d2*X_J3J4 + dE_d2*Z_J3J4 + dF_d2 ,
//        dG_d2*X_J3J4 + dH_d2*Z_J3J4 + dI_d2;
//    dX_d3 << dA_d3*X_J3J4 + dB_d3*Z_J3J4 + dC_d3 ,
//        dD_d3*X_J3J4 + dE_d3*Z_J3J4 + dF_d3 ,
//        dG_d3*X_J3J4 + dH_d3*Z_J3J4 + dI_d3;

//    Eigen::MatrixXd Jacobian_v(3,3);
//    Jacobian_v << dX_d1, dX_d2, dX_d3;

//    this->J << Jacobian_v;

//    this->Ji = J.inverse();

//    //TODO check this, print, be thorough!!!

//    //---------------------------------
//    // print
//    std::cout << " ********** In setJacobian ****************** " << std::endl;
//    std::cout << " Jacobian_v " << std::endl << Jacobian_v << std::endl;
//    std::cout << " Jacobian inverse " << std::endl << Ji << std::endl;
//    //---------------------------------

//    is_set_J = true;

//}

//bool TwoDotFiveDVS::setQdot_vl(){
//    //TODO check this, print, be thorough!!!

//    if(q_dot.rows()!=3)
//        q_dot.resize(3,1);

//    if(is_set_lambda & is_set_L & is_set_M & is_set_J & is_set_error){
//        //TODO confirm this
//        Eigen::MatrixXd Jc = this->L * (this->M).inverse() * this->J;

//        this->q_dot = -this->lambda * (Jc).inverse() * error;

//        //---------------------------------
//        // print
//        std::cout << " ********** In setQdot ****************** " << std::endl;
//        std::cout << " L " << std::endl << this->L << std::endl;
//        std::cout << " M inv " << std::endl << (this->M).inverse() << std::endl;
//        std::cout << " J " << std::endl << this->J << std::endl;
//        std::cout << " ---------- " << std::endl;
//        std::cout << " Jc " << std::endl << Jc << std::endl;
//        std::cout << " inverse Jc " << std::endl << (Jc).inverse() << std::endl;
//        std::cout << " lambda " << std::endl << this->lambda << std::endl;
//        std::cout << " error " << std::endl << error << std::endl;
//        std::cout << " q_dot " << std::endl << q_dot << std::endl;
//        //---------------------------------


//        // OR!!!!!

//        Eigen::VectorXd q_dot_V2;
//        q_dot_V2.resize(3);
//        q_dot_V2 = this->Ji * this->eef_v;

//        //---------------------------------
//        // print
//        std::cout << " ********** In setQdot version 2 ****************** " << std::endl;
//        std::cout << " inverse J " << std::endl << this->Ji << std::endl;
//        std::cout << " ve " << std::endl << eef_v << std::endl;
//        std::cout << " q_dot_V2 " << std::endl << q_dot_V2 << std::endl;
//        //---------------------------------

//        is_set_q_dot = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set q_dot matrix. Parameters required not set" << std::endl;
//        return false;
//    }
//}

//bool TwoDotFiveDVS::do_calculus_vl(){

//        set_error_vl();

//        setInteractionMatrix_vl();

//        setMmatrix_vl();

//        setCameraVelocity_vl();

//        setEEFvelFromCameraVelocities_vl();

//        setJacobian_vl();

//        setQdot_vl();

//}

//bool TwoDotFiveDVS::setJointAnglesToSend_vl(){

//    std::cout << "In setJointAnglesToSend_vl" << std::endl;

//    joint_angles_to_send[0] = joint_angles_received[0] + this->q_dot(0);
//    joint_angles_to_send[1] = joint_angles_received[1] + this->q_dot(1);
//    joint_angles_to_send[2] = joint_angles_received[2] + this->q_dot(2);

//    joint_angles_to_send[3] = joint_angles_received[3] ;
//    joint_angles_to_send[4] = joint_angles_received[4] ;
//    joint_angles_to_send[5] = joint_angles_received[5] ;

//    is_set_joint_angles_to_send = true;

//    std::cout << "out of setJointAnglesToSend_vl" << std::endl;
//    return true;
//}

//std::vector<double> TwoDotFiveDVS::getJointAnglesToSend_vl(){

//    setJointAnglesToSend_vl();
//    return joint_angles_to_send;
//}

//// ------------------------------------------------------------------------------
////                          LINEAR VELOCITY + ANGULAR VELOCITY
//// ------------------------------------------------------------------------------

//bool TwoDotFiveDVS::set_error(){

//    if(error.rows()!=6)
//        error.resize(6);

//    if(!is_set_theta_u)
//        setThetaU();

//    if(is_set_s_xyz & is_set_sStar_xyz & is_set_theta_u)
//    {

//        this->error(0) = this->s_xyz(0) - this->sStar_xyz(0);
//        this->error(1) = this->s_xyz(1) - this->sStar_xyz(1);
//        this->error(2) = log(this->p_xyz(2)/this->pStar_xyz(2));
//        this->error(3) = this->theta_u(0)*this->theta_u(1);
//        this->error(4) = this->theta_u(0)*this->theta_u(2);
//        this->error(5) = this->theta_u(0)*this->theta_u(3);
//        this->is_set_error = true;
//        //---------------------------------
//        // print
//        std::cout << " ********** ERROER ****************** " << std::endl;
//        std::cout << " error " << std::endl << error << std::endl;
//        //---------------------------------

//    }
//    else
//    {
//        std::cout << "Couldn't set error. Parameters required not set" << std::endl;
//        return false;
//    }
//}

//void TwoDotFiveDVS::setInteractionMatrix(){

//    if(L.cols()!= 6 & L.rows()!=6)
//    {
//        L.resize(6,6);
//        Lv.resize(3,3);
//        Lw.resize(3,3);
//    }

//    //TOCHECK please
//    //version with difference btw s and s* : not sure about this

//    std::cout << " ********** In setInteractionMatrix ****************** " << std::endl;

//    double x = this->error(0);
//    double y = this->error(1);
//    double Z = this->p_xyz(2); //TOCHECK please
//    double theta = this->theta_u(0); //TOCHECK this
//    std::vector<double> u;
//    u.push_back(this->theta_u(1));
//    u.push_back(this->theta_u(2));
//    u.push_back(this->theta_u(3));

//    // prints
//    std::cout << "x : " << x << " , "
//              << "y : " << y << " , "
//              << "Z : " << Z << std::endl;

//    std::cout << "theta : " << theta << std::endl;

//    std::cout << " u : " << std::endl;
//    for (int i =0; i< u.size(); i++)
//        std::cout << u[i] << " ";
//    std::cout << std::endl;

//    //
//    Eigen::MatrixXd Lthetau(3,3);

//    //compute Lv
//    this->Lv <<   -1      ,   0   ,   x  ,
//            0      ,   -1  ,    y  ,
//            0      ,   0   ,   -1;

//    //compute Lw
//    Eigen::Matrix3d sk_u = sk3x1(u);

//    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

//    double sinc_theta, sinc_thetaby2;
//    //TOCHECK put =0 when theta is 0
//    if (theta!=0)
//    {
//        sinc_theta = sin(theta)/theta;
//        sinc_thetaby2 = sin(theta/2)/(theta/2);
//    }
//    else
//    {
//        sinc_theta = sin(theta)/(theta+epsilon);
//        sinc_thetaby2 = sin(theta/2)/(theta/2+epsilon);
//    }

//    std::cout << "sinc_theta: " << sinc_theta << std::endl;
//    std::cout << "I3: \n" << I3 << std::endl;
//    std::cout << "sk_u: \n" << sk_u << std::endl;

//    std::cout << "(theta/2)*sk_u: \n" << (theta/2)*sk_u << std::endl;

//    std::cout << "(1 - sinc_theta/(sinc_theta*sinc_theta))*(sk_u*sk_u): \n" << (1 - sinc_theta/(sinc_theta*sinc_theta))*(sk_u*sk_u) << std::endl;


//    if (sinc_theta!=0)
//        Lthetau = I3 -(theta/2)*sk_u + (1 - sinc_theta/(sinc_thetaby2*sinc_thetaby2))*(sk_u*sk_u);
//    else
//        Lthetau = I3 -(theta/2)*sk_u + (1 - sinc_theta/(sinc_thetaby2*sinc_thetaby2+epsilon))*(sk_u*sk_u);

//    this->Lw << x*y          ,   -(1+x*x)    ,   y   ,
//            (1+y)*(1+y) ,   -x*y        ,   -x  ,
//            -y          ,   x           ,   0;
//    //compute L matrix
//    this->L << 1/Z*Lv , Lw  ,
//            Eigen::Matrix3d::Zero(), Lthetau;


//    std::cout << "Lv: " << std::endl << Lv << std::endl;
//    std::cout << "Lw: " << std::endl << Lw << std::endl;
//    std::cout << "Lthetau: " << std::endl << Lthetau << std::endl;

//    std::cout << "L: \n"<< L << std::endl;

//    is_set_L = true;

//}

//bool TwoDotFiveDVS::setCameraVelocity(){

//    if(cam_v.rows()!=6)
//        cam_v.resize(6);

//    //TODO
//    if (is_set_lambda & is_set_L  & is_set_error)
//    {

//       Eigen::MatrixXd L_inverse(6,6);

//       Eigen::Matrix3d l = (this->Lv).inverse();

//       L_inverse << l, -(l*(this->Lw)),
//               Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

//       std::cout << "Here" << std::endl;
//       this->cam_v = - this->lambda * L_inverse * this->error;
//       //---------------------------------
//       // print
//       std::cout << " ********** In setCameraVelocity ****************** " << std::endl;
//       std::cout << " L inverse " << std::endl << L_inverse << std::endl;
//       std::cout << " cam v " << std::endl << cam_v << std::endl;
//       //---------------------------------

//       //OLD : this->cam_v = - this->lambda * (this->L).inverse() * this->error;

//        is_set_cam_v = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set camera velocity. Parameters required not set" << std::endl;
//        return false;
//    }


//}

//bool TwoDotFiveDVS::setEEFvelFromCameraVelocities(){
//    if(eef_v.rows()!=6)
//        eef_v.resize(6);
//    if(is_set_cam_v & is_set_M)
//    {
//        this->eef_v << M * cam_v;

//        //---------------------------------
//        // print
//        std::cout << " ********** In setEEFvelFromCameraVelocities ****************** " << std::endl;
//        std::cout << " eef_v " << std::endl << eef_v << std::endl;
//        //---------------------------------

//        is_set_eef_v = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set eef velocity. Parameters required not set" << std::endl;
//        return false;
//    }

//}

//void TwoDotFiveDVS::setJacobian(){

////    if(J.rows()!= 6 & J.cols()!=6)
////    {
////        J.resize(6,6);
////        Ji.resize(6,6);
////    }

////    //TODO check this, print, be thorough!!!
////    for (int i =0; i< joint_state_.size(); i++)
////        std::cout << "joint_state " << i << " : "  << joint_state_[i] <<std::endl;

////    double s1 = sin(joint_state_[0]);
////    double c1 = cos(joint_state_[0]);
////    double s2 = sin(joint_state_[1]);
////    double c2 = cos(joint_state_[1]);
////    double s3 = sin(joint_state_[2]);
////    double c3 = cos(joint_state_[2]);

////    Eigen::Vector3d dx_dq1a, dx_dq1b, dx_dq2a, dx_dq2b, dx_dq3a, dx_dq3b;
////    dx_dq1a << -s1*c2*c3*A3+s1*s2*s3*A3-s1*c2*A2,
////            c1*c2*c3*A3-c1*s2*s3*A3+c1*c2*A2,
////            0;
////    dx_dq1b << s3*s1*c2*A4+c3*s1*s2*A4,
////            -s3*c1*c2*A4-c3*c1*s2*A4,
////            0;

////    dx_dq2a << -c1*s2*c3*A3-c1*c2*s3*A3-c1*s2*A2,
////            -s1*s2*c3*A3-s1*c2*s3*A3-s1*s2*A2,
////            -c2*c3*A3+s2*s3*A3-c2*A2;

////    dx_dq2b << s3*c1*s2*A4-c3*c1*c2*A4,
////            s3*s1*s2*A4-c3*s1*c2*A4,
////            s3*c2*A4+c3*s2*A4;

////    dx_dq3a << -c1*c2*s3*A3-c1*s2*c3*A3,
////            -s1*c2*s3*A3-s1*s2*c3*A3,
////            s2*s3*A3-c2*c3*A3;

////    dx_dq3b << -c3*c1*c2*A4-s3*c1*s2*A4,
////            -c3*s1*c2*A4+s3*s1*s2*A4,
////            c3*s2*A4+s3*c2*A4;

////    std::cout << "dx_dq1a " << dx_dq1a << std::endl;
////    std::cout << "dx_dq1b " << dx_dq1b << std::endl;
////    std::cout << "dx_dq2a " << dx_dq2a << std::endl;
////    std::cout << "dx_dq2b " << dx_dq2b << std::endl;
////    std::cout << "dx_dq3a " << dx_dq3a << std::endl;
////    std::cout << "dx_dq3b " << dx_dq3b << std::endl;

////    Eigen::Vector3d dx_dq1 = dx_dq1a + dx_dq1b;
////    Eigen::Vector3d dx_dq2 = dx_dq2a + dx_dq2b;
////    Eigen::Vector3d dx_dq3 = dx_dq3a + dx_dq3b;

////    Eigen::MatrixXd Jacobian_v(3,6);
////    Jacobian_v << dx_dq1, dx_dq2, dx_dq3, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
////    //TODO check joint_state previously

////    Eigen::MatrixXd Jacobian_w;

////    //TODO check the joints
////    Jacobian_w = this->robotstate_.getJacobianW(joint_state_);

////    this->J << Jacobian_v, Jacobian_w;

////    this->Ji = J.inverse();

////    //TODO check this, print, be thorough!!!

////    //---------------------------------
////    // print
////    std::cout << " ********** In setJacobian ****************** " << std::endl;
////    std::cout << " Jacobian_v " << std::endl << Jacobian_v << std::endl;
////    std::cout << " Jacobian_w " << std::endl << Jacobian_w << std::endl;
////    std::cout << " Jacobian inverse " << std::endl << Ji << std::endl;
////    //---------------------------------

////    is_set_J = true;

//}

//bool TwoDotFiveDVS::setQdot(){
//    //TODO check this, print, be thorough!!!

//    if(q_dot.rows()!=6)
//        q_dot.resize(6,1);

//    if(is_set_lambda & is_set_L & is_set_M & is_set_J & is_set_error){
//        //TODO confirm this
//        Eigen::MatrixXd Jc = this->L * (this->M).inverse() * this->J;

//        this->q_dot = -this->lambda * (Jc).inverse() * error;


//        //---------------------------------
//        // print
//        std::cout << " ********** In setQdot ****************** " << std::endl;
//        std::cout << " L " << std::endl << this->L << std::endl;
//        std::cout << " M inv " << std::endl << (this->M).inverse() << std::endl;
//        std::cout << " J " << std::endl << this->J << std::endl;
//        std::cout << " ---------- " << std::endl;
//        std::cout << " Jc " << std::endl << Jc << std::endl;
//        std::cout << " inverse Jc " << std::endl << (Jc).inverse() << std::endl;
//        std::cout << " lambda " << std::endl << this->lambda << std::endl;
//        std::cout << " error " << std::endl << error << std::endl;
//        std::cout << " q_dot " << std::endl << q_dot << std::endl;
//        //---------------------------------


//        // OR!!!!!

//        Eigen::VectorXd q_dot_V2;
//        q_dot_V2.resize(6);
//        q_dot_V2 = this->Ji * this->eef_v;

//        //---------------------------------
//        // print
//        std::cout << " ********** In setQdot version 2 ****************** " << std::endl;
//        std::cout << " inverse J " << std::endl << this->Ji << std::endl;
//        std::cout << " ve " << std::endl << eef_v << std::endl;
//        std::cout << " q_dot_V2 " << std::endl << q_dot_V2 << std::endl;
//        //---------------------------------

//        is_set_q_dot = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set q_dot matrix. Parameters required not set" << std::endl;
//        return false;
//    }
//}


//void TwoDotFiveDVS::set_R_ec(Eigen::MatrixXd r){
//    this->R_ec = r;
//    //---------------------------
//    // print
//    std::cout << "R_ec matrix :" << std::endl;
//    std::cout << R_ec << std::endl;
//    //---------------------------
//    is_set_R_ec = true;
//}

//void TwoDotFiveDVS::set_R_ec_star(Eigen::MatrixXd r){
//    this->R_ec_star = r;
//    //---------------------------
//    // print
//    std::cout << "R_ec_star matrix :" << std::endl;
//    std::cout << R_ec_star << std::endl;
//    //---------------------------
//    is_set_R_ec_star = true;
//}

//void TwoDotFiveDVS::set_R_0c(Eigen::MatrixXd r){
//    R_0c.resize(3,3);
//    this->R_0c =r;

//    //---------------------------
//    // print
//    std::cout << "R_0c matrix :" << std::endl;
//    std::cout << R_0c << std::endl;
//    //---------------------------

//    is_set_R_0c = true;
//}

//void TwoDotFiveDVS::set_d_ec(Eigen::VectorXd d){
//    d_ec.resize(3);
//    this->d_ec = d;

//    //---------------------------
//    // print
//    std::cout << "d_ec matrix :" << std::endl;
//    std::cout << d_ec << std::endl;
//    //---------------------------

//    is_set_d_ec = true;
//}

//void TwoDotFiveDVS::set_joints(std::vector<double> joints){
//    this->joint_state_[0] = joints[0];
//    this->joint_state_[1] = joints[1];
//    this->joint_state_[2] = joints[2];
//    this->joint_state_[3] = joints[3];
//    this->joint_state_[4] = joints[4];
//    this->joint_state_[5] = joints[5];

////    ROS_INFO("JOINTS ARE SET YEAH");
////    for (int i=0; i<6; i++){
////        std::cout << "joint " << i+1 << " , value: " <<this->joint_state_[i] << std::endl;
////    }
//}

//void TwoDotFiveDVS::setLambda(double l){
//    this->lambda = l;
//    this->is_set_lambda = true;
//}

//bool TwoDotFiveDVS::do_calculus(){

//    //TODO chek the interaction matrix

//        set_error();

//        setInteractionMatrix();

//        setMmatrix();

//        setCameraVelocity();

//       setEEFvelFromCameraVelocities();

//       setJacobian();

//       setQdot();

//}

//// ******************************************************************************
//// PRIVATE
//// ******************************************************************************

//bool TwoDotFiveDVS::setMmatrix(){

//    if(M.rows()!= 6 & M.cols()!=6)
//        M.resize(6,6);

//    if(is_set_R_0c & is_set_d_ec){
//        //TODO confirm this

//        std::vector<double> dec; dec.push_back(d_ec(0));
//        dec.push_back(d_ec(1));
//        dec.push_back(d_ec(2));

//        M << - R_0c                 , -R_0c*sk3x1(dec),
//                Eigen::Matrix3d::Zero(), - R_0c;

//        is_set_M = true;

//        //---------------------------------
//        //print
//        std::cout << "M" << std::endl;
//        std::cout << M << std::endl;
//        //---------------------------------
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set M matrix. Parameters required not set" << std::endl;
//        return false;
//    }

//}

//bool TwoDotFiveDVS::setMmatrix_vl(){

//    if(M.rows()!= 3 & M.cols()!=3)
//        M.resize(3,3);

//    if(is_set_R_0c){
//        //TODO confirm this

//        M << - R_0c;

//        is_set_M = true;

//        //---------------------------------
//        //print
//        std::cout << "M" << std::endl;
//        std::cout << M << std::endl;
//        //---------------------------------
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set M matrix. Parameters required not set" << std::endl;
//        return false;
//    }

//}

//bool TwoDotFiveDVS::setThetaU(){

//    if(theta_u.rows()!= 4)
//        theta_u.resize(4);

//    if(is_set_R_ec & is_set_R_ec_star){

//        Eigen::MatrixXd R = R_ec.inverse() * R_ec_star;

//        std::cout << "R to compute theta and u" << std::endl << R << std::endl;

//        std::cout << "trace of matrix : " << trace_of_matrix(R) << std::endl;

//        double t = trace_of_matrix(R);

//        //TOCHECK
//        if (t>1) t=1;
//        else if (t<-1) t=-1;
//        double theta = acos(t);

//        double u_x, u_y, u_z;
//        if (theta == 0)
//        {
//            // TOCHECK
//            std::cout << " Theta angle is 0." << std::endl;
//            u_x = 0;
//            u_y = 0;
//            u_z = 1;
//        }
//        else if(theta == M_PI)
//        {
//            //TODO
//            std::cout << " Theta angle is pi. \n This is going to crash" << std::endl;
//        }
//        else
//        {
//            u_x = 1/sin(2*theta) * (R(2,1)-R(1,2));
//            u_y = 1/sin(2*theta) * (R(0,2)-R(2,0));
//            u_z = 1/sin(2*theta) * (R(1,0)-R(0,1));
//        }

//        this->theta_u[0] = theta;
//        this->theta_u[1] = u_x;
//        this->theta_u[2] = u_y;
//        this->theta_u[3] = u_z;

//        //---------------------------------
//        //print
//        std::cout << "THETA AND U" << std::endl;
//        std::cout << " theta : " << theta << std::endl;
//        std::cout << " u_x : " << u_x << std::endl;
//        std::cout << " u_y : " << u_y << std::endl;
//        std::cout << " u_z : " << u_z << std::endl;
//        //---------------------------------

//        is_set_theta_u = true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set ThetaU matrix. Parameters required not set" << std::endl;
//        return false;
//    }
//}

//bool TwoDotFiveDVS::setJacobianInverse(){
//    if (is_set_J)
//    {
//        this->Ji = J.inverse();
//        is_set_Ji =  true;
//        return true;
//    }
//    else
//    {
//        std::cout << "Couldn't set jacobian inverse. Parameters required not set" << std::endl;
//        return false;
//    }

//}

//void TwoDotFiveDVS::putFlagsDown(){

//    is_set_s_xyz                =   false;
//    is_set_sStar_xyz            =   false;
//    is_set_L                =   false;
//    is_set_error            =   false;
//    is_set_lambda           =   false;
//    is_set_M                =   false;
//    is_set_cam_v            =   false;
//    is_set_eef_v            =   false;
//    is_set_J                =   false;
//    is_set_Ji               =   false;
//    is_set_R_ec             =   false;
//}

//Eigen::Matrix3d TwoDotFiveDVS::sk3x1(std::vector<double> p){
//    Eigen::Matrix3d sk_matrix;

//    if (p.size() != 3){
//        std::cout << "size of p not 3 (x,y,z). Returning ..." << std::endl;
//        // TODO devia mandar sair daqui
//    }

//    double X = p[0]; double Y = p[1]; double Z = p[2];
//    sk_matrix << 0  ,   -Z  ,    Y,
//            Z  ,   0   ,   -X,
//            -Y ,   X   ,    0;

//    return sk_matrix;
//}

//double TwoDotFiveDVS::trace_of_matrix(Eigen::MatrixXd r){
//    return (r(0,0) + r(1,1) + r(2,2));
//}

//Eigen::MatrixXd TwoDotFiveDVS::RotationFromUnitQuaternion(std::vector<double> q)
//{
//    double e0 = q[0];
//    double e1 = q[1];
//    double e2 = q[2];
//    double e3 = q[3];

//    Eigen::MatrixXd rotation_matrix(3,3);
//    rotation_matrix <<  1-2*(e2*e2+e3*e3) , 2*(e1*e2-e0*e3)     , 2*(e1*e3+e0*e2),
//                        2*(e1*e2+e0*e3)   , 1-2*(e1*e1+e3*e3)   , 2*(e2*e3-e0*e1),
//                        2*(e1*e3-e0*e2)   , 2*(e2*e3+e0*e1)     , 1-2*(e1*e1+e2*e2);

//    return rotation_matrix;
//}

//Eigen::MatrixXd TwoDotFiveDVS::rotationAroundZ(double theta){
//    Eigen::MatrixXd rot(3,3);
//    rot << cos(theta) , -sin(theta), 0.0,
//           sin(theta) , cos(theta) , 0.0,
//            0.0       , 0.0        , 1.0;
//    return rot;
//}

//Eigen::MatrixXd TwoDotFiveDVS::rotationAroundX(double theta){
//    Eigen::MatrixXd rot(3,3);
//    rot <<  1.0, 	 0.0, 	  0.0,
//            0.0, cos(theta), -sin(theta),
//            0.0, sin(theta),  cos(theta);
//    return rot;
//}

//Eigen::MatrixXd TwoDotFiveDVS::rotationAroundY(double theta){
//    Eigen::MatrixXd rot(3,3);
//    rot <<  cos(theta)  , 0.0   , sin(theta),
//            0.0         , 1.0   , 	 0.0    ,
//            -sin(theta) , 0.0   , cos(theta);
//    return rot;
//}

////bool TwoDotFiveDVS::getJointVectorFromJointState(std::vector<double> joints){
////    //TODO check joint state status
////    joints.clear();
////    joints.push_back(this->jointstate_.position[0]);
////    joints.push_back(this->jointstate_.position[1]);
////    joints.push_back(this->jointstate_.position[2]);
////    joints.push_back(this->jointstate_.position[3]);
////    joints.push_back(this->jointstate_.position[4]);
////    joints.push_back(this->jointstate_.position[5]);
////    return true;
////}
