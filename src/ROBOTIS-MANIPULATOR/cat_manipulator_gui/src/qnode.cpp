/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/cat_manipulator_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cat_manipulator_gui {

/***************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
    stop=true;
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
    ros::init(init_argc,init_argv,"cat_manipulator_gui");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

    robot_joints_pub = n.advertise<sensor_msgs::JointState>("/controller_joint_states",1);
    gripper_pub = n.advertise<std_msgs::Bool>("/driver_gripper/command", 1);
    robot_joints_sub = n.subscribe("/joint_states", 1, &QNode::get_joint_state_callback, this);


    start();
    return true;
}

void QNode::run() {
    int rate = 100;
    TrajectoryGenerator tragectoryGenrator;
    int internalRate = 1;
    //int refresh_rate = 2;
    double refresh_rate = 0.25; // works with very fast speed
    ros::Rate loop_rate(rate );
    double beginTime;

    while ( ros::ok() )
    {
      double secs =ros::Time::now().toSec();
      ros::spinOnce();
      callFK();

      double velocity = 0.04;
      std::cout << "time " <<secs<<std::endl;

      if(!stop){

        if(internalRate == 1){

          double duration = (goal_eef_pos - cur_eef_pos).norm()/velocity;
          beginTime = secs;

          ROS_INFO( "in time %f trajectory is created for the duration %f ", secs, duration);
          tragectoryGenrator.setLinear(cur_eef_pos, goal_eef_pos,duration);

          std::cout << "cur eef pos: " <<cur_eef_pos<<std::endl;
          std::cout << "goal Pos: " <<goal_eef_pos<<std::endl;

          std::cout << "goal_curr_dist:" << (goal_eef_pos - cur_eef_pos).norm()<< std::endl;
          std::cout << "===============================" <<std::endl;

        }

        double currTime = secs - beginTime;
        local_eef_pos = tragectoryGenrator.getLinearPosition(currTime);
        std::cout << "for the currtime: "<< currTime << " local Pos: " <<local_eef_pos<<std::endl;


        if ( (goal_eef_pos - cur_eef_pos).norm() <= 0.01){
                  ROS_INFO("========THE ROBOT IS IN GOAL POS");

                  local_eef_pos=goal_eef_pos;
                  std::cout << "goal_curr_dist" << (goal_eef_pos - cur_eef_pos).norm()<< std::endl;
                  std::cout << "local_curr_dist" << (local_eef_pos- cur_eef_pos).norm()<< std::endl;

                  //callIK();
                  std::cout << "cur eef pos: " <<cur_eef_pos <<std::endl;
                  std::cout << "goal Pos: " <<goal_eef_pos<<std::endl;
                  std::cout << "local Pos: " <<local_eef_pos<<std::endl;
        }


          ROS_INFO("========THE ROBOT IS MOVING");
          callIK();
          Eigen::VectorXd js(6);
            js= get_goal_joint_state();

            set_cur_joint_state(js);

            sensor_msgs::JointState msg;
            msg.header.stamp = ros::Time::now();
            int num_joints = 6;
            msg.name.resize(num_joints);
            msg.position.resize(num_joints);

            msg.name[0] = "joint1";
            msg.name[1] = "joint2";
            msg.name[2] = "joint3";
            msg.name[3] = "joint4";
            msg.name[4] = "joint5";
            msg.name[5] = "joint6";

            msg.position[0] = js(0);
            msg.position[1] = js(1);
            msg.position[2] = js(2);
            msg.position[3] = js(3);
            msg.position[4] = js(4);
            msg.position[5] = js(5);

            //ROS_INFO("should be publishing the joint states");


            send_jointposition_msgs( msg );

          internalRate++;

          if(internalRate > (rate/refresh_rate)){
            internalRate = 1;
          }
      }else{
        internalRate = 1;
        ROS_INFO("====== THE ROBOT IS STOPPED");


      }

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

    //log( Info , "Publish desired Joint States" );
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
    //ROS_INFO("Calling IK! ");
    Eigen::MatrixXd goal_rot_eef(3,3);

    Eigen::Matrix3d rollAngle(3,3);
    rollAngle << 1 , 0 , 0, 0,
        cos(goal_eef_zyxAngle(0)),-sin(goal_eef_zyxAngle(0)),
        0, sin(goal_eef_zyxAngle(0)), cos(goal_eef_zyxAngle(0));

    Eigen::Matrix3d pitchAngle(3,3);

    pitchAngle << cos(goal_eef_zyxAngle(1)) , 0, sin(goal_eef_zyxAngle(1)) ,
        0, 1,0,
        -sin(goal_eef_zyxAngle(1)),0, cos(goal_eef_zyxAngle(1));

    Eigen::Matrix3d yawAngle(3,3);
    yawAngle << cos(goal_eef_zyxAngle(2)), -sin(goal_eef_zyxAngle(2)), 0,
        sin(goal_eef_zyxAngle(2)), cos(goal_eef_zyxAngle(2)), 0,
        0, 0, 1 ;

    goal_rot_eef = yawAngle*pitchAngle*rollAngle;

    if( r.InverseKinematics( cur_joint_states_,local_eef_pos, goal_rot_eef, goal_joint_states_))
    {
        //goal_joint_states_(2) += M_PI/4; //old to be new uncomment
        ROS_INFO(" IK is possible" );
        if(r.checkGoalJointViabilityAndCorrect()){
          ROS_INFO("The calculated joint values are aout of bound");
        }

        return true;
    }
    else
    {
        //log( Info , " IK is not possible to achieve!!" );
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



}// namespace cat_manipulator_gui
