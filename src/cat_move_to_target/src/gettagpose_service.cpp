#include "ros/ros.h"
#include "cat_move_to_target/GetTagPose.h"
#include "cat_move_to_target/ApriltagsClass.hpp"
#include <fstream>

ApriltagsClass ctags;

std::vector<geometry_msgs::TransformStamped> tf_pose_array;

void addPoseToTFVector(const geometry_msgs::PoseStamped& pose_msg, int tag_id, std::string source_frame){

    //------------
    std::stringstream childframe_sstream;
    childframe_sstream << "tag" << tag_id;
    std::string child_frame = childframe_sstream.str();

    std::cout << "adding to vector the " << child_frame << std::endl;

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

    std::cout << "Size of vector : " << tf_pose_array.size() << std::endl;

    bool already_in_tfvector = false;

    //check if vector has any element
    if(!tf_pose_array.empty())
    {
        int i;
        for (i=0; i< tf_pose_array.size(); i++)
            if (  child_frame.compare((tf_pose_array[i]).child_frame_id) == 0)
            {
                tf_pose_array[i] = tf_pose;
                // check if has a given child_frame
                already_in_tfvector = true;
                break;
            }
    }

    if(!already_in_tfvector)
        tf_pose_array.push_back(tf_pose);

    std::cout <<  "*********** tf_pose_array *******" << std::endl;
    for (int j=0; j < tf_pose_array.size(); j++)
        std::cout << tf_pose_array[j] << std::endl;
}

void publishTf(){

    tf::TransformBroadcaster tfBr;
    if(!tf_pose_array.empty()){

        for (int i=0; i < tf_pose_array.size(); i++){
            tf_pose_array[i].header.stamp = ros::Time::now();
        }
        tfBr.sendTransform(tf_pose_array);

        //ROS_INFO("publishing ....");
    }

}

bool find_tag_pose( geometry_msgs::PoseStamped& pos, int tag_id){

    ctags.clearTempPose();

    if(ctags.getTagPoseByRequest(tag_id, pos)){
        std::cout << "********* pose ********* " << std::endl;
        std::cout << pos << std::endl;

        while(!ctags.checkTagStabilityByRequest(pos)){
            ROS_INFO("Checking tag pose.... ");
            ros::spinOnce();
            ctags.getTagPoseByRequest(tag_id, pos);
        }

        std::string source_frame;
        source_frame =  "camera_rgb_optical_frame";

        addPoseToTFVector(pos, tag_id, source_frame);

        ROS_INFO(" Advertised tag pose ") ;
        return true;
    }
    else
    {
        return false;
    }

}

// get camera matrix (read from file)
bool readCameraExtMatrix(Eigen::MatrixXd &T){

    std::string path = "/home/cat/catkin_ws/src/biovision/cat_robotcam_calibration/calibration_matrix";
    std::string filename = path + "/world_cam_matrix.txt" ;

    std::ifstream file(filename.c_str());
    std::string line;
    if(file.is_open()){

        getline (file,line);
        for (int i=0; i< 4; i++)
        {
          getline (file,line);
          std::stringstream ss(line.c_str());
          double a;
          int counter =0;
          while (ss >> a)
          {
            T(i, counter) = a;
            counter = counter+1;
          }
        }
        file.close();
        std::cout << " matrix : " << std::endl << T << '\n';
    }
    else
    {
        std::cout << "Reading file failed. Unable to open file." << std::endl;
        return false;
    }

    return true;

}

void convertToWorldPose(Eigen::Vector4d tag_pos, Eigen::Vector4d &world_pos, Eigen::MatrixXd T){

    world_pos = T * tag_pos;

    std::cout << "tag_pos : " << tag_pos << std::endl;
    std::cout << "world_pos : " << world_pos << std::endl;
    std::cout << "T : " << T << std::endl;
}

void convertWorldTBtoManip(Eigen::Vector4d src, Eigen::Vector4d &dst)
{
    Eigen::MatrixXd T_worldToManip(4,4);
    T_worldToManip << 1 , 0 , 0 , 0.135,
                      0 , 1 , 0 , 0.135,
                      0 , 0 , 1 , 0.023,
                      0 , 0 , 0 , 1;
    dst = (T_worldToManip).inverse() * src;
}


bool getTagPose(cat_move_to_target::GetTagPose::Request &req,
         cat_move_to_target::GetTagPose::Response &res)
{
    int tag_id = req.tag_id;
    std::vector<int> tags;
    tags.push_back(tag_id);

    ctags.init2(tags);

    geometry_msgs::PoseStamped pos;
    if(find_tag_pose(pos, tag_id))
    {
        std::cout << "pos \n " << pos << std::endl;

        res.foundtag = true;
        res.pos_x = pos.pose.position.x;
        res.pos_y = pos.pose.position.y;
        res.pos_z = pos.pose.position.z;
        res.ori_x = pos.pose.orientation.x;
        res.ori_y = pos.pose.orientation.y;
        res.ori_z = pos.pose.orientation.z;
        res.ori_w = pos.pose.orientation.w;

        //convert to world
        Eigen::MatrixXd T(4,4);
        Eigen::Vector4d tag_pos, world_pos, world_manip_pos;

        readCameraExtMatrix(T);

        tag_pos << pos.pose.position.x, pos.pose.position.y, pos.pose.position.z, 1.0;

        convertToWorldPose(tag_pos, world_pos, T);

        convertWorldTBtoManip(world_pos, world_manip_pos);

        res.w_pos_x = world_manip_pos(0);
        res.w_pos_y = world_manip_pos(1);
        res.w_pos_z = world_manip_pos(2);

        res.tag_id = tag_id;

        ROS_INFO("request: tag id = %d", req.tag_id);
        ROS_INFO("sending back response: \n [%3.7f %3.7f %3.7f] \n [%3.7f %3.7f %3.7f %3.7f]",
                 res.pos_x, res.pos_y, res.pos_z,
                 res.ori_x, res.ori_y, res.ori_z, res.ori_w);
        ROS_INFO("world coord: \n [%3.7f %3.7f %3.7f] ",
                 res.w_pos_x, res.w_pos_y, res.w_pos_z);
        return true;
    }
    else
    {
        ROS_INFO("Did not found tag");
        res.foundtag = false;
        res.pos_x = 0;
        res.pos_y = 0;
        res.pos_z = 0;
        res.ori_x = 0;
        res.ori_y = 0;
        res.ori_z = 0;
        res.ori_w = 0;
        res.w_pos_x = 0;
        res.w_pos_y = 0;
        res.w_pos_z = 0;
        return true;
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "get_tag_pose");

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_tag_pose", getTagPose);
    ROS_INFO("Ready to send tag poses");

    ctags.startClass(argc, argv);

    while(ros::ok()){
        ros::spinOnce();
        publishTf();
    }

    return 0;
}
