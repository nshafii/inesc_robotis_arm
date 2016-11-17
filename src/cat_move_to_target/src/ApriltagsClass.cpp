
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/cat_move_to_target/ApriltagsClass.hpp"
#include "cat_move_to_target/GetTagPose.h"
#include <iostream>
#include <map>

/*****************************************************************************
** Implementation [AprilTagsClass]
*****************************************************************************/

ApriltagsClass::ApriltagsClass(){}
ApriltagsClass::ApriltagsClass(int argc, char** argv){
    //initialize vector and stuff here

    ros::init(argc, argv, "detect_tags");
    hasnewtag = false;
}

ApriltagsClass::~ApriltagsClass(){
    if(ros::isStarted()){
        //ros::shutdown(); //confirm
        ros::waitForShutdown();
    }
    wait();
}

void ApriltagsClass::startClass(int argc, char** argv){
    //initialize vector and stuff here

    ros::init(argc, argv, "detect_tags");
    hasnewtag = false;
}

bool ApriltagsClass::init(std::vector<int> id){

    ROS_INFO("Initialization ");

    setTargetsID(id);

    ros::NodeHandle nh;
    tags_detection_sub = nh.subscribe("/apriltags/detections", 1, &ApriltagsClass::tags_detection_callback, this);

    tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tags_pose",1);

    while(ros::ok()){

        ros::spinOnce();

    }

    return true;

}

bool ApriltagsClass::init2(std::vector<int> id){

    ROS_INFO("Initialization ");

    setTargetsID(id);

    ros::NodeHandle nh;
    tags_detection_sub = nh.subscribe("/apriltags/detections", 1, &ApriltagsClass::tags_detection_callback_srvlike, this);

    tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tags_pose",1);
    return true;

}

bool ApriltagsClass::init_srvlike(int argc, char** argv, std::vector<int> id){

    ros::init(argc, argv, "detect_tags");

    ROS_INFO(" Initializing the Tag Class ");

    setTargetsID(id);

    ros::NodeHandle nh;
    tags_detection_sub = nh.subscribe("/apriltags/detections", 1, &ApriltagsClass::tags_detection_callback_srvlike, this);

    ros::spin();

    return true;

}

bool ApriltagsClass::init_noIDinput(){

    ros::NodeHandle nh;
    tags_detection_sub = nh.subscribe("/apriltags/detections", 1, &ApriltagsClass::tags_detection_callback_srvlike, this);

    tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("tags_pose",1);

    //in case of searching for the cylinder tag
    this->previous_tag_cylinder.pose.position.x = -1000;
    this->previous_tag_cylinder.pose.position.y = -1000;
    this->previous_tag_cylinder.pose.position.z = -1000;

    ROS_INFO(" subscriber to apriltags initialized");
    return true;
}

bool ApriltagsClass::getTagIds() {
    int detection_id;
    std::vector<int> tags;
    apriltags::AprilTagDetection tag_detection;

    for (unsigned int i=0; i< ((detection_msg_storage_.detections).size()); ++i)
    {
        tag_detection = (detection_msg_storage_.detections)[i];
        detection_id = tag_detection.id;
        tags.push_back(detection_id);

        std::cout << tag_detection << std::endl;
    }

    this->target_tags_id = tags;
    //   for(int i=0; i< tags.size(); i++)
    //       std::cout << tags[i] << std::endl;

    if(tags.size() > 0)
        return true;
    else
    {
        ros::spinOnce();
        return false;
    }
}

bool ApriltagsClass::setTargetsID(std::vector<int> targets_id){

    ROS_INFO(" Setting tags id ");
    for (int i=0; i< targets_id.size(); ++i)
    {
        target_tags_id.push_back(targets_id[i]);
        //std::cout << target_tags_id[i] << std::endl;
    }

    target_tags_pose_array.resize(targets_id.size());

    temp_pose_array.resize(targets_id.size());
    for (int i=0; i< target_tags_pose_array.size(); ++i){
        //initialize temp_pose
        temp_pose_array[i].pose.orientation.x = 0;
        temp_pose_array[i].pose.orientation.y = 0;
        temp_pose_array[i].pose.orientation.z = 0;
        temp_pose_array[i].pose.orientation.w = 0;
        temp_pose_array[i].pose.position.x = 0;
        temp_pose_array[i].pose.position.y = 0;
        temp_pose_array[i].pose.position.z = 0;
    }
    return true;
}

bool ApriltagsClass::checkForTagStability(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2, double error_allowed)
{

    ROS_INFO(" Checking for stability ... ");

    if(fabs(pose1.pose.position.x - pose2.pose.position.x) < error_allowed &
            fabs(pose1.pose.position.y - pose2.pose.position.y) < error_allowed &
            fabs(pose1.pose.position.z - pose2.pose.position.z) < error_allowed &
            fabs(pose1.pose.orientation.x - pose2.pose.orientation.x) < error_allowed &
            fabs(pose1.pose.orientation.y - pose2.pose.orientation.y) < error_allowed &
            fabs(pose1.pose.orientation.z - pose2.pose.orientation.z) < error_allowed &
            fabs(pose1.pose.orientation.w - pose2.pose.orientation.w) < error_allowed)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ApriltagsClass::checkIfHasEmptyPose(std::vector<geometry_msgs::PoseStamped> temp_pose){

    for (int i = 0; i<temp_pose.size(); i++)
    {
        if ((temp_pose[i]).pose.orientation.x == 0 && (temp_pose[i]).pose.orientation.y == 0 &&
                (temp_pose[i]).pose.orientation.z == 0 && (temp_pose[i]).pose.orientation.w  == 0 &&
                (temp_pose[i]).pose.position.x == 0 && (temp_pose[i]).pose.position.y == 0 && (temp_pose[i]).pose.position.z == 0)
        { return true;}

    }
    return false;
}

void ApriltagsClass::poseStampedToTf(const geometry_msgs::PoseStamped& pose_msg, std::string child_frame, std::string source_frame){

    tf2_ros::StaticTransformBroadcaster sTfBr;

    //------------
    ROS_INFO("Sending the tf!");
    geometry_msgs::TransformStamped tf_pose;

    tf_pose.transform.translation.x = pose_msg.pose.position.x;
    tf_pose.transform.translation.y = pose_msg.pose.position.y;
    tf_pose.transform.translation.z = pose_msg.pose.position.z;
    tf_pose.transform.rotation.x = pose_msg.pose.orientation.x;
    tf_pose.transform.rotation.y = pose_msg.pose.orientation.y;
    tf_pose.transform.rotation.z = pose_msg.pose.orientation.z;
    tf_pose.transform.rotation.w = pose_msg.pose.orientation.w;
    tf_pose.header.stamp = ros::Time::now();
    tf_pose.header.frame_id = source_frame;
    tf_pose.child_frame_id = child_frame;
    sTfBr.sendTransform(tf_pose);

    //    std::cout <<  "*********** POSE  *******" << std::endl;
    //    std::cout << tf_pose << std::endl;

    std::cout << "ENtrei aqui " << std::endl;

    ros::Duration(5.0).sleep();

    ROS_INFO("publishing .... %s to %s frame", source_frame.c_str(), child_frame.c_str());

    int tag_id = 5;
    if(!getTagPoseInWorld(tag_id))
        ROS_INFO("Could not find tag5");

    tag_id = 2;
    if(!getTagPoseInWorld(tag_id))
        ROS_INFO("Could not find tag2");
}

bool ApriltagsClass::getTagPoseInWorld(int tag_id){

    tf::TransformListener listener;
    tf::StampedTransform transformListen;

    std::string tag_frame;
    std::stringstream ss;
    ss << "tag" <<tag_id;
    tag_frame = ss.str();

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TransformStamped tf_geomMsg;

    //ros::Duration(5.0).sleep();

    try{
        listener.waitForTransform( tag_frame, "world", ros::Time(), ros::Duration(1.0) );
        listener.lookupTransform( tag_frame, "world", ros::Time(), transformListen);
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    listener.waitForTransform(tag_frame, "world", ros::Time(), ros::Duration(1.0) );
    listener.lookupTransform(tag_frame, "world", ros::Time(), transformListen);
    tf::transformStampedTFToMsg(transformListen, tf_geomMsg);

    pose_msg.pose.position.x = tf_geomMsg.transform.translation.x;
    pose_msg.pose.position.y = tf_geomMsg.transform.translation.y;
    pose_msg.pose.position.z = tf_geomMsg.transform.translation.z;
    pose_msg.pose.orientation.x = tf_geomMsg.transform.rotation.x;
    pose_msg.pose.orientation.y = tf_geomMsg.transform.rotation.y;
    pose_msg.pose.orientation.z = tf_geomMsg.transform.rotation.z;
    pose_msg.pose.orientation.w = tf_geomMsg.transform.rotation.w;

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = tf_geomMsg.header.frame_id;

    tag_pose_pub.publish(pose_msg);

    if(tag_id == 2)
    {
        ROS_INFO(" ---------------------------------- ");
        ROS_INFO(" ****** TAG ***** ");
        std::cout << tag_frame << std::endl;
        std::cout << pose_msg << std::endl;
        ROS_INFO(" ----------------------------------");
    }

    return true;
}

bool ApriltagsClass::listenToTF(std::string child_frame, std::string source_frame,  geometry_msgs::PoseStamped& pose_msg){


    std::cout << "Entrei no listenToTF" << std::endl;
    tf::TransformListener listener;
    tf::StampedTransform transformListen;
    geometry_msgs::TransformStamped tf_geomMsg;

    ros::spinOnce();
    try{
        listener.waitForTransform( child_frame, source_frame, ros::Time(), ros::Duration(10.0) );
        listener.lookupTransform( child_frame, source_frame, ros::Time(), transformListen);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    listener.waitForTransform(child_frame, source_frame, ros::Time(), ros::Duration(1.0) );
    listener.lookupTransform(child_frame, source_frame, ros::Time(), transformListen);
    tf::transformStampedTFToMsg(transformListen, tf_geomMsg);

    pose_msg.pose.position.x = tf_geomMsg.transform.translation.x;
    pose_msg.pose.position.y = tf_geomMsg.transform.translation.y;
    pose_msg.pose.position.z = tf_geomMsg.transform.translation.z;
    pose_msg.pose.orientation.x = tf_geomMsg.transform.rotation.x;
    pose_msg.pose.orientation.y = tf_geomMsg.transform.rotation.y;
    pose_msg.pose.orientation.z = tf_geomMsg.transform.rotation.z;
    pose_msg.pose.orientation.w = tf_geomMsg.transform.rotation.w;

    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = tf_geomMsg.header.frame_id;

    ROS_INFO(" ---------------------------------- ");
    ROS_INFO(" ****** TAG ***** ");
    std::cout << "Child: " << child_frame << std::endl;
    std::cout << "Source: " << source_frame << std::endl;
    std::cout << pose_msg << std::endl;
    ROS_INFO(" ----------------------------------");


    //check if this is the best way to do so
    tag_pose_pub.publish(pose_msg);

    return true;
}


void ApriltagsClass::tags_detection_callback(const apriltags::AprilTagDetections::ConstPtr& msg){

    ROS_INFO(" ****** detection ***** ");

    int detection_id;
    std::vector<int>::iterator it;
    apriltags::AprilTagDetection tag_detection;

    double error_allowed = 0.002;
    bool found_id;
    int counter;

    if(target_tags_id.size()>0)
    {

        for (unsigned int i=0; i< ((msg->detections).size()); ++i)
        {
            geometry_msgs::PoseStamped tag_pose;
            tag_detection = (msg->detections)[i];
            detection_id = tag_detection.id;

            found_id = false;
            counter =0;

            for (std::vector<int>::iterator it = target_tags_id.begin() ; it != target_tags_id.end(); it++, counter++)
                if(*it == detection_id)
                {
                    found_id = true;
                    break;
                }

            if(found_id)
            {
                //std::cout << " ****** In Tag " << target_tags_id[counter] << "*****" << std::endl;
                // retrieve pose
                tag_pose.pose = tag_detection.pose;

                //check for stability
                //temp_pose @ same place as the target_tags_id , i.e, at it (not *it)
                if (detection_id ==2)
                    error_allowed = 0.01;
                if(checkForTagStability(temp_pose_array[counter], tag_pose, error_allowed))
                {
                    //add to pose vector in it place
                    tag_pose.header = msg->header;
                    target_tags_pose_array[counter] = tag_pose;

                    //std::cout <<  "********** TAG ID *******" << std::endl;
                    //std::cout << target_tags_id[counter] << std::endl;

                    std::stringstream ss ; ss << "tag" << detection_id;
                    std::string child_frame = ss.str();
                    poseStampedToTf(tag_pose, child_frame, "camera_rgb_optical_frame");

                    temp_pose_array[counter] = tag_pose;
                }
                else
                {
                    temp_pose_array[counter] = tag_pose;
                }

            }

        }

    }
    else
    {
        std::cout << "No target set. Ending" << std::endl;
    }

}

void ApriltagsClass::tags_detection_callback_srvlike(const apriltags::AprilTagDetections::ConstPtr& msg){

    detection_msg_storage_.header = msg->header;
    detection_msg_storage_.detections = msg->detections;

    hasnewtag = true;

}

bool ApriltagsClass::getAllTagsPose(std::map<int, geometry_msgs::PoseStamped> &pos){

    //TODO CHECK THIS
    pos.clear();
    geometry_msgs::PoseStamped p;
    for (int i =0; i< this->target_tags_id.size(); i++)
    {
        getTagPoseByRequest(target_tags_id[i], p);
        pos[target_tags_id[i]] =p;

        //        std::cout  << "pos[" << target_tags_id[i]  << "] \n"
        //                   << pos[target_tags_id[i]] << std::endl;
    }
}

bool ApriltagsClass::checkCylinderTagStability(int tag_id, geometry_msgs::PoseStamped &pos){
    //check if stable
    if(    this->previous_tag_cylinder.pose.position.x == -1000 &
           this->previous_tag_cylinder.pose.position.y == -1000 &
           this->previous_tag_cylinder.pose.position.z == -1000)
    {
        previous_tag_cylinder = pos;
        return false;
    }

    geometry_msgs::PoseStamped p_cur, p_prv ;
    std::cout << tag_id << " => " << pos << '\n';

    p_cur = pos;
    p_prv = previous_tag_cylinder;

    if(!checkForTagStability(p_cur,p_prv, 0.001))
    {
        ROS_INFO("Not stable");

        this->previous_tag_cylinder = pos;
        return false;
    }
    std::cout << "Tag " << tag_id << "stable! " <<std::endl;

    return true;

}

bool ApriltagsClass::checkAllTagsStability(std::map<int, geometry_msgs::PoseStamped> &pos){

    //check if stable
    if(this->previous_tag_map.size() == 0 )
    {
        ROS_INFO("Here");
        this->previous_tag_map = pos;
        return false;
    }

    geometry_msgs::PoseStamped p_cur, p_prv ;
    for (std::map<int, geometry_msgs::PoseStamped>::iterator it = pos.begin(); it != pos.end(); it++)
    {

        std::cout << it->first << " => " << it->second << '\n';
        p_cur = it->second;
        p_prv = this->previous_tag_map[it->first];

        if(!checkForTagStability(p_cur,p_prv, 0.001))
        {
            ROS_INFO("Not stable");

            this->previous_tag_map = pos;
            return false;
        }
        std::cout << "Tag " << it->first << "stable! " <<std::endl;
    }

    ROS_INFO("ALL TAGS STABLE.");

    return true;


    //if not previous map = this

}

bool ApriltagsClass::getTagPoseByRequest(int tag_id, geometry_msgs::PoseStamped& pos){

    ros::spinOnce();
    ROS_INFO(" ****** Retrieving tag pose ***** ");
    // This do not check for tag stability

    hasnewtag = false;
    // changes in tags_detection_callback_srvlike
    while(hasnewtag == false){
        ros::spinOnce();
    }

    apriltags::AprilTagDetection tag_detection;

    int detection_id;


    std::vector<int> tags_found;
    for (unsigned int i=0; i< ((detection_msg_storage_.detections).size()); ++i)
    {
        tag_detection = (detection_msg_storage_.detections)[i];
        tags_found.push_back(tag_detection.id);
    }


    std::vector<int>::iterator iter = std::find(tags_found.begin(), tags_found.end(), tag_id);
    size_t index = std::distance(tags_found.begin(), iter);
    if(index == tags_found.size())
    {
        return false;
    }

    tag_detection = (detection_msg_storage_.detections)[index];
    detection_id = tag_detection.id;

    pos.header = detection_msg_storage_.header;
    pos.pose = tag_detection.pose;

    return true;

}


void ApriltagsClass::clearTempPose(){
    temp_pose.pose.position.x = 0 ;
    temp_pose.pose.position.y = 0 ;
    temp_pose.pose.position.z = 0 ;
    temp_pose.pose.orientation.x = 0 ;
    temp_pose.pose.orientation.y = 0 ;
    temp_pose.pose.orientation.z = 0 ;
    temp_pose.pose.orientation.w = 0 ;
}

bool ApriltagsClass::checkTagStabilityByRequest(geometry_msgs::PoseStamped pos){

    hasnewtag = false;
    // changes in tags_detection_callback_srvlike
    while(hasnewtag == false){
        ros::spinOnce();
    }

    if(temp_pose.pose.position.x == 0 & temp_pose.pose.position.y == 0 & temp_pose.pose.position.z == 0 &
            temp_pose.pose.orientation.x == 0 & temp_pose.pose.orientation.y == 0 &
            temp_pose.pose.orientation.z == 0 & temp_pose.pose.orientation.w == 0 )
    {
        temp_pose.header = pos.header;
        temp_pose.pose = pos.pose;
        ros::spinOnce();
        return false;
    }
    else
    {
        if(checkForTagStability(pos, temp_pose, 0.0005))
            return true;
        else
        {
            temp_pose.header = pos.header;
            temp_pose.pose = pos.pose;
            ros::spinOnce();
            return false;
        }
    }
}

void ApriltagsClass::publishTfTagPose(const geometry_msgs::PoseStamped& pose_msg, std::string child_frame, std::string source_frame){

    tf2_ros::StaticTransformBroadcaster sTfBr;

    //------------
    ROS_INFO("Sending the tf!");
    geometry_msgs::TransformStamped tf_pose;

    tf_pose.transform.translation.x = pose_msg.pose.position.x;
    tf_pose.transform.translation.y = pose_msg.pose.position.y;
    tf_pose.transform.translation.z = pose_msg.pose.position.z;
    tf_pose.transform.rotation.x = pose_msg.pose.orientation.x;
    tf_pose.transform.rotation.y = pose_msg.pose.orientation.y;
    tf_pose.transform.rotation.z = pose_msg.pose.orientation.z;
    tf_pose.transform.rotation.w = pose_msg.pose.orientation.w;
    tf_pose.header.stamp = ros::Time::now();
    tf_pose.header.frame_id = source_frame;
    tf_pose.child_frame_id = child_frame;
    sTfBr.sendTransform(tf_pose);

    std::cout <<  "*********** publishTfTagPose *******" << std::endl;
    std::cout << tf_pose << std::endl;

    ros::Duration(3.0).sleep();

    ROS_INFO("publishing .... %s to %s frame", source_frame.c_str(), child_frame.c_str());
}

void ApriltagsClass::publishTagPoseArray(std::map<int, geometry_msgs::PoseStamped> tag_map){
    //using child_frame calibtag !!!!
    //------------
    ROS_INFO("Sending the tf!");
    std::vector<geometry_msgs::TransformStamped> tag_tf_vector;
    geometry_msgs::TransformStamped tag_tf;
    std::string source_frame, child_frame;
    source_frame = "camera_rgb_optical_frame";

    geometry_msgs::PoseStamped ps;

    for (std::map<int, geometry_msgs::PoseStamped>::iterator it = tag_map.begin(); it != tag_map.end(); it++)
    {
        std::cout << it->first << " => " << it->second << '\n';
        std::stringstream ss;  ss << "calibtag" << it->first;
        child_frame = ss.str();
        ps = it->second;

        tag_tf.transform.translation.x = ps.pose.position.x;
        tag_tf.transform.translation.y = ps.pose.position.y;
        tag_tf.transform.translation.z = ps.pose.position.z;
        tag_tf.transform.rotation.x = ps.pose.orientation.x;
        tag_tf.transform.rotation.y = ps.pose.orientation.y;
        tag_tf.transform.rotation.z = ps.pose.orientation.z;
        tag_tf.transform.rotation.w = ps.pose.orientation.w;
        tag_tf.header.stamp = ros::Time::now();
        tag_tf.header.frame_id = source_frame;
        tag_tf.child_frame_id = child_frame;
        tag_tf_vector.push_back(tag_tf);
    }

    tf2_ros::StaticTransformBroadcaster sTfBr;
    sTfBr.sendTransform(tag_tf_vector);

    //    std::cout <<  "*********** Publish all tags pose *******" << std::endl;
    //    for (int i =0; i < tag_tf_vector.size(); i++)
    //        std::cout << tag_tf_vector[i] << std::endl;

    ros::Duration(3.0).sleep();

    ROS_INFO("publishing .... %s to %s frame", source_frame.c_str(), child_frame.c_str());
}

std::vector<geometry_msgs::PoseStamped> ApriltagsClass::getTagPoseArray(){
    return this->target_tags_pose_array;
}

bool ApriltagsClass::getTagPose(geometry_msgs::PoseStamped pos, int id){

    bool found_id;
    int counter =0 ;
    for (std::vector<int>::iterator it = target_tags_id.begin() ; it != target_tags_id.end(); it++, counter++)
        if(*it == id)
        {
            found_id = true;
            break;
        }

    if(found_id)
    {
        pos = target_tags_pose_array[counter] ;
    }

    return found_id;
}

bool ApriltagsClass::computeLLSUsingSVD(){
    Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
    std::cout << "Here is the matrix A:\n" << A << std::endl;
    Eigen::VectorXf b = Eigen::VectorXf::Random(3);
    A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::cout << "Here is the right hand side b:\n" << b << std::endl;
    std::cout << "The least-squares solution is:\n"
              << A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << std::endl;


    //      Eigen::VectorXd answ; answ.resize(A.cols());
    //      answ = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

