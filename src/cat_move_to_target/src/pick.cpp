//#include "../include/cat_move_to_target/pick.hpp"
//#include <iostream>

///*****************************************************************************
//** Implementation [AprilTagsClass]
//*****************************************************************************/

//Pick::Pick(){

//    moveit::planning_interface::MoveGroup group("manipulator");

//    std::string plannerId;
//    plannerId = "KPIECEkConfigDefault";
//    group.setPlannerId(plannerId.c_str());
//    group.allowReplanning(true);

//    // setrobotjointsuccess = false;
//    // newGoal = false;


//    // register goal and feedback callbacks

//    //start assyncronous cenas + subscriptions and publishers




//}

//Pick::~Pick(){}


//void Pick::printPose(geometry_msgs::Pose& pos){
//    std::cout << " X = " << pos.position.x
//              << " Y = " << pos.position.y
//              << " Z = " << pos.position.z
//              << " Rx = " << pos.orientation.x
//              << " Ry = " << pos.orientation.y
//              << " Rz = " << pos.orientation.z
//              << " Rw = " << pos.orientation.w
//              << std::endl;
//}

//void Pick::preemptedCB(){
//    ROS_INFO("%s: Preempted", action_name_.c_str());
//    std::cerr << "-----------------------------------------------------------------------------------------" << std::endl;
//    std::cerr << "---------------------------------------preempt-------------------------------------------" << std::endl;
//    std::cerr << "-----------------------------------------------------------------------------------------" << std::endl;
//    // set the action state to preempted
//    as_.setPreempted();
//}

//void analysisCB(){

//    if(as_.isActive())
//        return;

//    bool pickPart = true;

//    tf::TransformListener listener;
//    static tf::StampedTransform transformListen;

//    bool success;

//    tf::Quaternion q;



//}














