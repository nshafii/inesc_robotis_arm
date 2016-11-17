//#ifndef cat_move_to_target_MOVE_TO_POINT_H
//#define cat_move_to_target_MOVE_TO_POINT_H

///*****************************************************************************
//** Includes
//*****************************************************************************/

//#include <ros/ros.h>

//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <tf2_ros/static_transform_broadcaster.h>

//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <moveit_msgs/PlanningScene.h>
//#include <moveit/collision_detection/collision_matrix.h>
//#include <moveit/collision_detection/collision_robot.h>
//#include <moveit/collision_detection/world.h>
//#include <moveit/warehouse/planning_scene_world_storage.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit_msgs/ExecuteKnownTrajectory.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/GetPositionIK.h>

//#include <control_msgs/FollowJointTrajectoryActionResult.h>
//#include <actionlib/server/simple_action_server.h>

//#include <cat_common/robot_common.hpp>



///*****************************************************************************
//** Class [MoveToPose]
//*****************************************************************************/

//class Pick{

//public:
//    Pick(int argc, char** argv);
//    virtual ~Pick();
//    bool init(std::vector<int> id);

//protected:

//    control_msgs::FollowJointTrajectoryActionResult TrajectoryFinished;
//    int TrajectoryFinishedNr;

//    ros::NodeHandle nh_;

//    actionlib::SimpleActionServer<Pick> as_;
//    std::string action_name;

//    pick::PickGoal goal_;
//    pick::PickFeedback feedback_;
//    pick::PickResult result_;

//    ros::Subscriber robotCmdVel_Subs;
//    ros::Subscriber robotFollowJointTrajectoryResultState_Subs;
//    unsigned int FollowJointTrajectoryResultState_State;
//    ros::ServiceClient setar;
//    ros::ServiceServer service_MoveRobotJoints;
//    pick::SetJointValuesRequest reqJ;
//    pick::SetJointValuesResponse resJ;
//    pick::SetJointValues set_srv;
//    std::string followTrajectoryNodeName;
//    std::vector <bool> robotShoulderElbowWrist;

//    int state;

//    tf2_ros::StaticTransformBroadcaster Sbr;
//    geometry_msgs::TransformStamped tf_Pose;
//    std::vector <geometry_msgs::TransformStamped> tf_pickPoses;

//    //obj to pick
//    ////double ObjLength;
//    ////double ToolLength;
//    ////double PickSlack;
//    ////double SlackBetweenObjTable;
//    ////double EndEffectorZoffsetToTool;

//};





//#endif
