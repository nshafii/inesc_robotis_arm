#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>

#include <cat_move_to_target/RobotState.hpp>


geometry_msgs::PoseStamped loadPose(){

    std::vector<double> p_1_p(3);
    std::vector<double> p_1_o(4);

    std::vector<double> p_2_p(3);
    std::vector<double> p_2_o(4);

    p_1_p[0]= 0.111066 ;
    p_1_p[1]= 0.285683 ;
    p_1_p[2]= 0.331531 ;
    p_1_o[0]= -0.563051 ;
    p_1_o[1]= 0.822958 ;
    p_1_o[2]= 0.0426677 ;
    p_1_o[3]= 0.0623912 ;

    // 1.2 , 0, 0.785, 0 , 1.42 , 0

    p_2_p[0]= -0.304234 ;
    p_2_p[1]= -0.161317;
    p_2_p[2]= 0.33619 ;
    p_2_o[0]= 0.79135 ;
    p_2_o[1]= -0.603053 ;
    p_2_o[2]= -0.0557928 ;
    p_2_o[3]= 0.0835433 ;

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = p_2_p[0];
    pos.pose.position.y = p_2_p[1];
    pos.pose.position.z = p_2_p[2];
    pos.pose.orientation.x = p_2_o[0];
    pos.pose.orientation.y = p_2_o[1];
    pos.pose.orientation.z = p_2_o[2];
    pos.pose.orientation.w = p_2_o[3];

    std::cout << pos << std::endl;
    return pos;
}

int main( int argc, char** argv){

    ros::init(argc, argv, "ik_manipulator");

    ros::NodeHandle n;

    std::cout << "Starting ... " << std::endl;


    geometry_msgs::PoseStamped pose_test = loadPose();


    RobotState r;

    std::vector<double> ik_joint_values;

//    bool InverseKinematics(Eigen::Vector3d pos, Eigen::MatrixXd rot, Eigen::VectorXd &joint_angles);

//    r.InverseKinematics(pose_test, ik_joint_values);

    ROS_INFO("This node is not working. Check it");
    ros::shutdown();

    ros::spin();
    return 0;

}
