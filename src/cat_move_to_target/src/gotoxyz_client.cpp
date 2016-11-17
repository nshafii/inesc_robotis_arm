#include "ros/ros.h"
#include <cstdlib>
#include "cat_move_to_target/GetJointValues.h"
#include "Eigen/Dense"

Eigen::MatrixXd rotationMatrixFromZYXeuler(std::vector<double> zyx_eulerAngles)
{
    //TODO check this
    double alpha = zyx_eulerAngles[0];
    double beta = zyx_eulerAngles[1];
    double gamma = zyx_eulerAngles[2];

    double c_a = cos(alpha); double s_a = sin(alpha);
    double c_b = cos(beta); double s_b = sin(beta);
    double c_g = cos(gamma); double s_g = sin(gamma);

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  c_a*c_b     , c_a*s_b*s_g-s_a*c_g   , c_a*s_b*c_g+s_a*s_g,
                        s_a*c_b     , s_a*s_b*s_g+c_a*c_g   , s_a*s_b*c_g-c_a*s_g,
                        -s_b        , c_b*s_g               , c_b*c_g;

    std::cout << "rotation_matrix" << std::endl;
    std::cout << rotation_matrix << std::endl;

    return rotation_matrix;
}

 std::vector<double> loadPose(int getFirstPose){

    std::vector<double> p_1_p(6);
    std::vector<double> p_2_p(6);

    p_1_p[0]= 0 ;
    p_1_p[1]= -0.411 ;
    p_1_p[2]= 0.4530 ;
    p_1_p[3]= -M_PI/2;
    p_1_p[4]= 0;
    p_1_p[5]= 0;

    // -pi/2, 0, 0, 0, 0, 0

    p_2_p[0]= 0 ;
    p_2_p[1]= -0.645 ;
    p_2_p[2]= 0.159 ;
    p_2_p[3]= -M_PI/2;
    p_2_p[4]= 0;
    p_2_p[5]= 0;

     // -pi/2, pi/2, -pi/2, 0, 0, 0

    if(getFirstPose)
    {
        return p_1_p;
    }
    else
    {
        return p_2_p;
    }

}



int main(int argc, char** argv){

    ros::init(argc, argv, "get_joint_values_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<cat_move_to_target::GetJointValues>("gotoxyz_service/getJointValues");

    std::vector<double> p(6);

    cat_move_to_target::GetJointValues srv;

    p = loadPose(true);

    std::vector<double> zyx_eulerAngles(3);
    zyx_eulerAngles[0] = p[3];
    zyx_eulerAngles[1] = p[4];
    zyx_eulerAngles[2] = p[5];
    rotationMatrixFromZYXeuler(zyx_eulerAngles);

    srv.request.pos_x = p[0];
    srv.request.pos_y = p[1];
    srv.request.pos_z = p[2];
    srv.request.zyx_angle_z = p[3];
    srv.request.zyx_angle_y = p[4];
    srv.request.zyx_angle_x = p[5];
    srv.request.hardcoded_wrist = false;

    if(client.call(srv)){
         ROS_INFO("Theta1: %3.4f", srv.response.theta1);
         ROS_INFO("Theta2: %3.4f", srv.response.theta2);
         ROS_INFO("Theta3: %3.4f", srv.response.theta3);
         ROS_INFO("Theta4: %3.4f", srv.response.theta4);
         ROS_INFO("Theta5: %3.4f", srv.response.theta5);
         ROS_INFO("Theta6: %3.4f", srv.response.theta6);
    }
    else
    {
        ROS_ERROR("Failed to call service ");
        return 1;
    }


    ROS_INFO("new pose");
    p = loadPose(false);

    client.shutdown();

    srv.request.pos_x = p[0];
    srv.request.pos_y = p[1];
    srv.request.pos_z = p[2];
    srv.request.zyx_angle_z = p[3];
    srv.request.zyx_angle_y = p[4];
    srv.request.zyx_angle_x = p[5];
    srv.request.hardcoded_wrist = false;

    if(client.call(srv)){
         ROS_INFO("Theta1: %3.4f", srv.response.theta1);
         ROS_INFO("Theta2: %3.4f", srv.response.theta2);
         ROS_INFO("Theta3: %3.4f", srv.response.theta3);
         ROS_INFO("Theta4: %3.4f", srv.response.theta4);
         ROS_INFO("Theta5: %3.4f", srv.response.theta5);
         ROS_INFO("Theta6: %3.4f", srv.response.theta6);
    }
    else
    {
        ROS_ERROR("Failed to call service ");
        return 1;
    }

    return 0;
}
