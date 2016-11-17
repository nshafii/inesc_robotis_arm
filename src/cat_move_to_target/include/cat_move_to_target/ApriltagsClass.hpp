#ifndef cat_move_to_target_CILINDER_DETECTOR_APRILTAGS_H
#define cat_move_to_target_CILINDER_DETECTOR_APRILTAGS_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <apriltags/AprilTagDetections.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <sstream>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>

/*****************************************************************************
** Class [ApriltagsClass]
*****************************************************************************/

class ApriltagsClass{

public:
    ApriltagsClass();
    ApriltagsClass(int argc, char** argv);
    void startClass(int argc, char** argv);
    virtual ~ApriltagsClass();
    bool init(std::vector<int> id);
    bool init2(std::vector<int> id);
    bool init_srvlike(int argc, char** argv, std::vector<int> id);
    bool init_noIDinput();

    void tags_detection_callback(const apriltags::AprilTagDetections::ConstPtr& msg);
    void tags_detection_callback_srvlike(const apriltags::AprilTagDetections::ConstPtr& msg);

    bool getTagPoseByRequest(int tag_id, geometry_msgs::PoseStamped& pos);
    bool getTagIds();
    bool getAllTagsPose(std::map<int, geometry_msgs::PoseStamped> &pos);
    bool checkAllTagsStability(std::map<int, geometry_msgs::PoseStamped> &pos);
    bool checkCylinderTagStability(int tag_id, geometry_msgs::PoseStamped &pos);

    bool checkTagStabilityByRequest(geometry_msgs::PoseStamped pos);
    void publishTfTagPose(const geometry_msgs::PoseStamped& pose_msg, std::string child_frame, std::string source_frame);
    void publishTagPoseArray(std::map<int, geometry_msgs::PoseStamped> tag_map);

    bool setTargetsID(std::vector<int> targets_id);

    std::vector<geometry_msgs::PoseStamped> getTagPoseArray();
    bool getTagPose(geometry_msgs::PoseStamped pos, int id);

    void clearTempPose();

    bool listenToTF(std::string child_frame, std::string source_frame, geometry_msgs::PoseStamped& pose_msg);

    bool computeLLSUsingSVD();

    std::vector<int> returnTagsIds(){return this->target_tags_id;}

private:

    bool getTagPoseInWorld(int tag_id);

    void poseStampedToTf(const geometry_msgs::PoseStamped& pose_msg, std::string child_frame, std::string source_frame);

    bool checkIfHasEmptyPose(std::vector<geometry_msgs::PoseStamped> temp_pose_array);
    bool checkForTagStability(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2, double error_allowed);

    bool hasnewtag;

    geometry_msgs::PoseStamped previous_tag_cylinder;
    std::map<int, geometry_msgs::PoseStamped> previous_tag_map;
    std::vector<int> target_tags_id;

    std::vector<geometry_msgs::PoseStamped> target_tags_pose_array;
    std::vector<geometry_msgs::PoseStamped> temp_pose_array;

    apriltags::AprilTagDetections detection_msg_storage_;
    geometry_msgs::PoseStamped temp_pose;

    ros::Subscriber tags_detection_sub;

    ros::Publisher tag_pose_pub;

};





#endif // cat_move_to_target_CILINDER_DETECTOR_APRILTAGS_H
