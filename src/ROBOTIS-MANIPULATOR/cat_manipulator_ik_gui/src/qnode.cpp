/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/cat_manipulator_ik_gui/qnode.hpp"
#define ZYZ_Y_FUZZ (0.000001)

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cat_manipulator_ik_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {

    cur_joint_states_.resize(NUM_JOINTS);
    goal_joint_states_.resize(NUM_JOINTS);
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"cat_manipulator_ik_gui");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    robot_joints_pub = n.advertise<sensor_msgs::JointState>("/controller_joint_states",1);
    gripper_pub = n.advertise<std_msgs::Bool>("/driver_gripper/command", 1);
    robot_joints_sub = n.subscribe("/joint_states", 1, &QNode::get_joint_state_callback, this);

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

void QNode::logInfo(const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] : " << msg;

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log( const LogLevel &level, const std::string &msg) {
    logging_model.insertRows(logging_model.rowCount(),1);
    std::stringstream logging_model_msg;
    switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] : " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] : " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] : " << msg;
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

//cat
void QNode::send_jointposition_msgs(const sensor_msgs::JointState& goalJointStates)
{
    robot_joints_pub.publish(goalJointStates);

    log( Info , "Publish desired Joint States" );
}

void QNode::send_grippercommand_msgs(const std_msgs::Bool& gripper_command)
{
    (this->gripper_pub).publish(gripper_command);

    log( Info , "Publish desired Gripper Command" );
}

void QNode::get_joint_state_callback(const sensor_msgs::JointState::ConstPtr& current_joint_states)
{
    Eigen::VectorXd js(6);
    js << current_joint_states->position[0],
            current_joint_states->position[1],
            current_joint_states->position[2],
            current_joint_states->position[3],
            current_joint_states->position[4],
            current_joint_states->position[5];

    cur_joint_states_ = js;
    callFK();
}

Eigen::Vector3d  matZyzConvert(Eigen::MatrixX3d m)
{
  Eigen::Vector3d zyz;
  zyz[1] = atan2(sqrt(pow(m(0,2),2) + pow(m(1,2),2)), m(2,2));

  if (fabs(zyz(1)) < ZYZ_Y_FUZZ)
  {
    zyz[0] = 0.0;
    zyz[1] = 0.0;               /* force Y to 0 */
    zyz[2] = atan2(-m(1,0), m(0,0));
  }
  else if (fabs(zyz(1) - M_PI) < ZYZ_Y_FUZZ)
  {
    zyz[0] = 0.0;
    zyz[1] = M_PI;             /* force Y to 180 */
    zyz[2] = atan2(m(1,0), -m(0,0));
  }
  else
  {
    zyz[0] = atan2(m(2,1), m(2,0));
    zyz[2] = atan2(m(1,2), -m(0,2));
  }

  return zyz;
}

Eigen::MatrixX3d rx( double s )
{
    Eigen::MatrixX3d R(3,3);

    R << 1.0, 	 0.0, 	  0.0,
         0.0, cos(s), -sin(s),
         0.0, sin(s),  cos(s);

    return R;
}

Eigen::MatrixX3d ry( double s )
{
    Eigen::MatrixX3d R(3,3);

    R << cos(s), 0.0, sin(s),
         0.0, 	 1.0, 	 0.0,
        -sin(s), 0.0, cos(s);

    return R;
}

Eigen::MatrixX3d rz( double s )
{
    Eigen::MatrixX3d R(3,3);

    R << cos(s), -sin(s), 0.0,
         sin(s),  cos(s), 0.0,
            0.0,     0.0, 1.0;

    return R;
}

Eigen::MatrixX3d rpy2rotation( double r, double p, double y )
{
    Eigen::MatrixX3d R = rz(y)*ry(p)*rx(r);
    //Eigen::MatrixX3d R = rx(r)*ry(p)*rz(y);
    return R;
}

Eigen::Quaternionf rpy2quaternion( double r, double p, double y )
{
    Eigen::MatrixX3d R = rpy2rotation(r, p, y);
    // todo danger
    //Eigen::Quaternionf q(R);
    Eigen::Quaternionf q;
    return q;
}


void twoaxisrot(double r11, double r12, double r21, double r31, double r32, Eigen::Vector3d res){
  res[0] = atan2( r11, r12 );
  res[1] = acos ( r21 );
  res[2] = atan2( r31, r32 );
}

Eigen::Vector3d quaternion2zyz( Eigen::Quaternionf q )
{
  Eigen::Vector3d res;

  twoaxisrot( 2*(q.y()*q.z() - q.w()*q.x()),
                    2*(q.x()*q.z() + q.w()*q.y()),
                    q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
                    2*(q.y()*q.z() + q.w()*q.x()),
                   -2*(q.x()*q.z() - q.w()*q.y()),
                   res);

  return res;
}

bool QNode::callFK(Eigen::VectorXd joint_states, Eigen::VectorXd &eef)
{
    // update the current eef pose.
    Eigen::MatrixXd rot_eef(3,3);
    Eigen::Vector3d eef_pos;
    r.ForwardKinematics(joint_states, eef_pos, rot_eef);

    Eigen::Vector3d zyxAngle(3);
    r.rotationMatrixToZYXeuler(rot_eef, zyxAngle);

    eef << eef_pos , zyxAngle;

    return true;
}

bool QNode::callFK()
{
    // update the current eef pose.
    Eigen::MatrixXd cur_rot_eef(3,3);
    r.ForwardKinematics(cur_joint_states_, cur_eef_pos, cur_rot_eef);

    r.rotationMatrixToZYXeuler(cur_rot_eef, cur_eef_zyxAngle);

    return true;
}

bool QNode::callIK()
{
    ROS_INFO("Calling IK! ");
    Eigen::MatrixXd goal_rot_eef(3,3);

/*
    std::vector<double> zyx_angles(3);
    zyx_angles[0] = goal_eef_zyxAngle(0);
    zyx_angles[1] = goal_eef_zyxAngle(1);
    zyx_angles[2] = goal_eef_zyxAngle(2);
    goal_rot_eef= r.rotationMatrixFromZYXeuler(zyx_angles);
*/


    //Eigen::Vector3d euZYZ = quaternion2zyz((rpy2quaternion(goal_eef_zyxAngle(0),goal_eef_zyxAngle(1),goal_eef_zyxAngle(2))));

    Eigen::Vector3d euZYZ = matZyzConvert(rpy2rotation(goal_eef_zyxAngle(0),goal_eef_zyxAngle(1),goal_eef_zyxAngle(2)));

    Eigen::Matrix3d rollAngle(3,3);

    rollAngle << cos(euZYZ(0)), -sin(euZYZ(0)), 0,
    sin(euZYZ(0)), cos(euZYZ(0)), 0,
    0, 0, 1 ;
    //
    Eigen::Matrix3d pitchAngle(3,3);
    //
    pitchAngle << cos(euZYZ(1)) , 0, sin(euZYZ(1)) ,
        0, 1,0,
        -sin(euZYZ(1)),0, cos(euZYZ(1));
    //
    Eigen::Matrix3d yawAngle(3,3);
    yawAngle << cos(euZYZ(2)), -sin(euZYZ(2)), 0,
        sin(euZYZ(2)), cos(euZYZ(2)), 0,
        0, 0, 1 ;

//    Eigen::Matrix3d rollAngle(3,3);
//    rollAngle << cos(goal_eef_zyxAngle(0)), -sin(goal_eef_zyxAngle(0)), 0,
//        sin(goal_eef_zyxAngle(0)), cos(goal_eef_zyxAngle(0)), 0,
//        0, 0, 1 ;
//
//    Eigen::Matrix3d pitchAngle(3,3);
//
//    pitchAngle << cos(goal_eef_zyxAngle(1)) , 0, sin(goal_eef_zyxAngle(1)) ,
//        0, 1,0,
//        -sin(goal_eef_zyxAngle(1)),0, cos(goal_eef_zyxAngle(1));
//
//    Eigen::Matrix3d yawAngle(3,3);
//    yawAngle << cos(goal_eef_zyxAngle(2)), -sin(goal_eef_zyxAngle(2)), 0,
//        sin(goal_eef_zyxAngle(2)), cos(goal_eef_zyxAngle(2)), 0,
//        0, 0, 1 ;

    goal_rot_eef = rollAngle* pitchAngle* yawAngle;
    //goal_rot_eef =  yawAngle * pitchAngle * rollAngle;
    printJointState(); // for debugin

    if( r.InverseKinematics( cur_joint_states_, goal_eef_pos, goal_rot_eef, goal_joint_states_))
    {
        //goal_joint_states_(2) += M_PI/4; //old to be new uncomment
        log( Info , " IK is possible" );

        return true;
    }
    else
    {
        log( Info , " IK is not possible to achieve!!" );
        return false;
    }

}

void QNode::printSTARTmessage(){

    logInfo( "************************** " );
    logInfo( "Please Click on Start !!! " );
    logInfo( "************************** " );

}

void QNode::printEEFState(){

    std::stringstream log_msgs;

    log( Info , "************* EEF STATE*************  : " );
    log_msgs << " \n "
             << " x : "  << cur_eef_pos(0)<< " \n "
             << " y : "  << cur_eef_pos(1)<< " \n "
             << " z : "  << cur_eef_pos(2)  << " \n "
             << " z_ANGLE:"  << cur_eef_zyxAngle(0) << " \n "
             << " y_ANGLE:"  << cur_eef_zyxAngle(1) << " \n "
             << " x_ANGLE:"  << cur_eef_zyxAngle(2) << " \n "
             << " \n ";
    log( Info , log_msgs.str() );

}

void QNode::printEEFState(Eigen::VectorXd eef){

    std::stringstream log_msgs;

    log( Info , "************* EEF STATE*************  : " );
    log_msgs << " \n "
             << " x : "  << eef(0)<< " \n "
             << " y : "  << eef(1)<< " \n "
             << " z : "  << eef(2)  << " \n "
             << " z_ANGLE:"  << eef(3) << " \n "
             << " y_ANGLE:"  << eef(4) << " \n "
             << " x_ANGLE:"  << eef(5) << " \n "
             << " \n ";
    log( Info , log_msgs.str() );

}

void QNode::printJointState(){

    log( Info , "************* JOINT STATE*************  : " );
    std::stringstream log_msgs;

    log_msgs << " \n "
             << "   joint1 : "  << cur_joint_states_(0)<< " \n "
             << "   joint2 : "  << cur_joint_states_(1)<< " \n "
             << "   joint3 : "  << cur_joint_states_(2) << " \n "
             << "   joint4 : "  << cur_joint_states_(3)<< " \n "
             << "   joint5 : "  << cur_joint_states_(4)<< " \n "
             << "   joint6 : "  << cur_joint_states_(5)  << " \n ";

    log( Info , log_msgs.str() );
}

Eigen::VectorXd QNode::get_cur_joint_state(){
    return cur_joint_states_;
}

void QNode::set_cur_joint_state(Eigen::VectorXd  js){
    cur_joint_states_ = js;
}

Eigen::VectorXd QNode::get_goal_joint_state(){
    return goal_joint_states_ ;
}

void QNode::set_goal_joint_state(Eigen::VectorXd d){
    goal_joint_states_ = d;
}

Eigen::Vector3d QNode::get_cur_eef_pos(){
    return cur_eef_pos;
}

void QNode::set_goal_eef_pos(Eigen::Vector3d d){
    goal_eef_pos = d;
}

Eigen::Vector3d QNode::get_cur_eef_zyxAngle(){
    return cur_eef_zyxAngle;
}

void QNode::set_goal_eef_zyxAngle(Eigen::Vector3d d){
    goal_eef_zyxAngle = d;
}



}// namespace cat_manipulator_ik_gui
