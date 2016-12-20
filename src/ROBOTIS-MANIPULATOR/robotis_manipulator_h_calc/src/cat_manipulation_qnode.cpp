
#include <ros/ros.h>
#include <string>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

//#define M_PI 3.1415926535897932384626433832795

Eigen::Quaterniond rpy2quaternion(double r, double p, double y);

class ManipulatorMotion {
private :

    int init_argc;
    char** init_argv;

    ros::Publisher send_fk_msgs_pub;
    ros::Publisher send_ik_msgs_pub;

    ros::Subscriber end_effector_pose_sub;


public:
    ManipulatorMotion(int argc, char** argv );
    bool init();
    void run();

    void send_fk_msgs(std_msgs::Bool msgs );
    void send_ik_msgs(geometry_msgs::Pose msgs);
    void end_effector_pose_callback(const geometry_msgs::Pose::ConstPtr& msgs );


    void update_curr_pos(double x, double y, double z);
    void update_curr_ori(double x, double y, double z, double w);

};

ManipulatorMotion::ManipulatorMotion(int argc, char **argv): init_argc(argc), init_argv(argv){}

bool ManipulatorMotion::init(){
    ros::init(init_argc, init_argv, "cat_manipulator_move_node");

    ros::start();
    ros::NodeHandle n;

    send_fk_msgs_pub = n.advertise<std_msgs::Bool>("/robotis_manipulator_h/fk_msgs", 0);
    send_ik_msgs_pub = n.advertise<geometry_msgs::Pose>("/robotis_manipulator_h/ik_msgs", 0);

    end_effector_pose_sub = n.subscribe("/robotis_manipulator_h/end_effector_pose", 1, &ManipulatorMotion::end_effector_pose_callback, this);

    start();
    return true;
}

void ManipulatorMotion::run() {
    ros::Rate loop_rate(50);
    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ManipulatorMotion::send_fk_msgs(std_msgs::Bool msgs){
    send_fk_msgs_pub.publish( msgs );

    log( Info , "Solve Forward Kinematics" );
}

void ManipulatorMotion::send_ik_msgs(geometry_msgs::Pose msgs){
    send_ik_msgs_pub.publish(msgs);

    ROS_INFO("Solve Inverse Kinematics");
    ROS_INFO("End effector desired pose: ");

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

     ROS_INFO(  log_msgs.str() );

}

void ManipulatorMotion::end_effector_pose_callback(const geometry_msgs::Pose::ConstPtr &msgs)
{
    ROS_INFO("End effector Current Pose : ");

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

     ROS_INFO(  log_msgs.str() );

     Eigen::Quaterniond QR(msgs.orientation.w,msgs.orientation.x,msgs.orientation.y,msgs.orientation.z);

     Eigen::MatrixXd R = QR.toRotationMatrix();

     double roll = atan2( R.coeff(2,1), R.coeff(2,2) ) * 180.0 / M_PI;
     double pitch = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) ) * 180.0 / M_PI;
     double yaw = atan2 ( R.coeff(1,0) , R.coeff(0,0) ) * 180.0 / M_PI;

     std::stringstream log_msgs_rpy;
     log_msgs_rpy << " \n "
                  << "des. pos. roll: "
                  << roll << " \n "
                  << "des. pos. pitch : "
                  << pitch << " \n "
                  << "des. pos. yaw : "
                  << yaw << " \n ";

      ROS_INFO(  log_msgs_rpy.str() );

}

void ManipulatorMotion::printFK()
{
    std_msgs::Bool msgs;

    msgs.data = true;

    send_fk_msgs( msgs );

}

void ManipulatorMotion::chooseDesiredEndEffectorPose(){
    geometry_msgs::Pose msgs;

    // x,y,z, r_deg, p_deg, y_deg
    double desired_pose [6]  = {0.300235, 0.146877, 0.591150, -2.639978, 1.377765, -2.238868};

    msgs.position.x = desired_pose[0];
    msgs.position.y = desired_pose[1];
    msgs.position.z = desired_pose[2];

    double roll = desired_pose[3] * M_PI/180;
    double roll = desired_pose[3] * M_PI/180;
    double roll = desired_pose[3] * M_PI/180;

    Eigen::Quaterniond QR= rpy2quaternion(roll, pitch, yaw);

    msgs.orientation.x = QR.x();
    msgs.orientation.y = QR.y();
    msgs.orientation.z = QR.z();
    msgs.orientation.w = QR.w();

    send_ik_msgs(msgs);

}

void ManipulatorMotion::printEndEffectorPose(){

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

Eigen::Quaterniond rpy2quaternion(double r, double p, double y){

    // rpy 2 rotation
    Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

    Eigen::Matrix3d R_plus;
    R_plus = R.block(0,0,3,3);

    Eigen::Quaterniond QR;
    QR = R_plus;

    return QR;
}






