#include <ros/ros.h>
#include "../include/cat_move_to_target/ApriltagsClass.hpp"
#include "cat_move_to_target/GetTagPose.h"


#define MANIPULATOR_TAG_ID 6
#define CILINDER_TAG_ID 2

ros::ServiceClient client;

bool listenToTF(std::string child_frameid, std::string source_frameid,  geometry_msgs::PoseStamped& pose_msg){

    geometry_msgs::TransformStamped tf_geomMsg;
    tf::TransformListener listener;
    tf::StampedTransform transformListen;

    bool found_tf = false;

    try{
        listener.waitForTransform(source_frameid, child_frameid, ros::Time(), ros::Duration(10.0) );
        listener.lookupTransform(source_frameid, child_frameid, ros::Time(), transformListen);
        found_tf = true;
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        found_tf = false;
    }

    if(found_tf){

        // to make it stable, retrieve again again

        listener.waitForTransform(source_frameid, child_frameid, ros::Time(0), ros::Duration(5.0) );
        listener.lookupTransform(source_frameid, child_frameid, ros::Time(0), transformListen);
        tf::transformStampedTFToMsg(transformListen, tf_geomMsg);

        pose_msg.pose.position.x = tf_geomMsg.transform.translation.x;
        pose_msg.pose.position.y = tf_geomMsg.transform.translation.y;
        pose_msg.pose.position.z = tf_geomMsg.transform.translation.z;
        pose_msg.pose.orientation.x = tf_geomMsg.transform.rotation.x;
        pose_msg.pose.orientation.y = tf_geomMsg.transform.rotation.y;
        pose_msg.pose.orientation.z = tf_geomMsg.transform.rotation.z;
        pose_msg.pose.orientation.w = tf_geomMsg.transform.rotation.w;

        std::cout << "In listenToTf ...."  << std::endl;
        std::cout << pose_msg << std::endl;
        return true;
    }
    else
        return false;
}


bool call_tagpose_srv(int tag_id){
    cat_move_to_target::GetTagPose srv;
    srv.request.tag_id = tag_id;

    ROS_INFO(" tag id %d", tag_id);
    if(client.call(srv)){

        if(srv.response.foundtag){
            ROS_INFO("pos_x: %3.7f", srv.response.pos_x);
            ROS_INFO("pos_y: %3.7f", srv.response.pos_y);
            ROS_INFO("pos_z: %3.7f", srv.response.pos_z);
            ROS_INFO("ori_x: %3.7f", srv.response.ori_x);
            ROS_INFO("ori_y: %3.7f", srv.response.ori_y);
            ROS_INFO("ori_y: %3.7f", srv.response.ori_z);
            ROS_INFO("ori_w: %3.7f", srv.response.ori_w);
            ROS_INFO("TAG ID: %d", srv.response.tag_id);

            geometry_msgs::PoseStamped pos;

            std::stringstream childframe_sstream;
            childframe_sstream << "tag" << tag_id;
            std::string child_frame = childframe_sstream.str();

            if(!listenToTF(child_frame, "world", pos) );
                    listenToTF(child_frame, "world", pos);

        }
        else
        {
            ROS_INFO("TAG %d not found", tag_id);
        }
    }
    else
    {
        ROS_INFO("Failed to call service ");
        return false;
    }
}


int main (int argc, char** argv) {

    ros::init(argc, argv, "detect_tags");

    ros::NodeHandle nh;

    ApriltagsClass ctags(argc, argv);
    std::vector<int> id;
    id.push_back(CILINDER_TAG_ID);
    id.push_back(MANIPULATOR_TAG_ID);
    ctags.init2(id);

    client = nh.serviceClient<cat_move_to_target::GetTagPose>("get_tag_pose");

    int next = 1;

    while (ros::ok())
    {
        std::cout << "Press 1 to go to pose and 0 to leave : ";
        std::cin >> next;

        if (next == 0)
        {
            ros::shutdown();
        }

        if(next == 1)
        {

            ROS_INFO("cilinder tag");
            call_tagpose_srv(CILINDER_TAG_ID);
            ROS_INFO("****");
            ROS_INFO("manipulator tag");
            call_tagpose_srv(MANIPULATOR_TAG_ID);

        }
        std::cout << "y ? " << std::endl;
    }

    return 0;
}
