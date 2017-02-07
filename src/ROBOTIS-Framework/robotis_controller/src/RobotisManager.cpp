/*
 * RobotisManager.cpp
 *
 *  Created on: 2015. 11. 18.
 *      Author: zerom
 */


#include <ros/ros.h>
#include <iostream>

#include <unistd.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/ControlWrite.h"
#include "robotis_controller_msgs/ControlTorque.h"
#include "robotis_controller_msgs/PublishPosition.h"

#include "../../robotis_controller/include/handler/GroupHandler.h"
#include "../../robotis_controller/include/RobotisController.h"

using namespace ROBOTIS;
using namespace std;
RobotisController   *controller = new RobotisController();
GroupHandler        grp_handler(controller);

pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;
int                 syncwrite_addr;
int                 syncwrite_data_length;
std::vector <unsigned char> syncwrite_param;

std::vector <int>   publish_list;

int get_id_from_name(std::string name)
{
    int id = -1;

    if("_ALL_" == name)
        return 254;

    for(int i = 0; i < controller->idList.size(); i++)
    {
        id = controller->idList[i];
        if(controller->getDevice(id)->getJointName() == name)
            return id;
    }
    return -1;
}

void publish_position_callback(const robotis_controller_msgs::PublishPosition::ConstPtr& msg)
{
    std::cout << "publish_position callback" << std::endl;
    if(msg->name.size() == 0 || msg->name.size() != msg->publish.size())
        return;

    for(int i = 0; i < msg->name.size(); i++)
    {
        int id = get_id_from_name(msg->name[i]);
        if(id == -1)
            continue;

        if(msg->publish[i] == true) {
            grp_handler.pushBulkRead(id, controller->getDevice(id)->ADDR_PRESENT_POSITION);
            if ( std::find(publish_list.begin(), publish_list.end(), id) == publish_list.end() )
                publish_list.push_back(get_id_from_name(msg->name[i]));
        }
        else {
            grp_handler.deleteBulkRead(id);
            std::vector<int>::iterator iter = std::find(publish_list.begin(), publish_list.end(), id);
            if(iter != publish_list.end())
                publish_list.erase(iter);
        }
    }
}

void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::cout << "joint_state callback" << std::endl;
    static int old_sec;
    int n = 0, id = -1;

    if(msg->name.size() == 0 || msg->name.size() != msg->position.size())
        return;

    pthread_mutex_lock(&mutex);
    syncwrite_param.clear();
    for(unsigned int idx = 0; idx < msg->name.size(); idx++)
    {
        id = get_id_from_name(msg->name[idx]);
        if(id != -1)
        {
            syncwrite_addr = controller->getDevice(id)->ADDR_GOAL_POSITION;
            syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
            int pos     = controller->getDevice(id)->rad2Value(msg->position[idx]);
            syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1);
            syncwrite_param[n++]  = id;
            if(syncwrite_data_length == 2)
            {
                syncwrite_param[n++]  = DXL_LOBYTE(pos);
                syncwrite_param[n++]  = DXL_HIBYTE(pos);
            }
            else if(syncwrite_data_length == 4)
            {
                syncwrite_param[n++]  = DXL_LOBYTE(DXL_LOWORD(pos));
                syncwrite_param[n++]  = DXL_HIBYTE(DXL_LOWORD(pos));
                syncwrite_param[n++]  = DXL_LOBYTE(DXL_HIWORD(pos));
                syncwrite_param[n++]  = DXL_HIBYTE(DXL_HIWORD(pos));
            }
        }
    }
    pthread_mutex_unlock(&mutex);
}

void control_write_callback(const robotis_controller_msgs::ControlWrite::ConstPtr& msg)
{
    std::cout << "control_write callback" << std::endl;
    switch(msg->length)
    {
    case 1:
        controller->write(get_id_from_name(msg->name), msg->addr, msg->value, LENGTH_1BYTE, 0);
        break;
    case 2:
        controller->write(get_id_from_name(msg->name), msg->addr, msg->value, LENGTH_2BYTE, 0);
        break;
    case 4:
        controller->write(get_id_from_name(msg->name), msg->addr, msg->value, LENGTH_4BYTE, 0);
        break;
    default:
        break;
    }
}

void control_torque_callback(const robotis_controller_msgs::ControlTorque::ConstPtr& msg)
{
    std::cout << "control_torque callback" << std::endl;
    int n = 0, id = -1;

    if(msg->name.size() == 0 || msg->name.size() != msg->enable.size())
        return;

    pthread_mutex_lock(&mutex);
    syncwrite_param.clear();
    for(int i = 0; i < msg->name.size(); i++)
    {
        id = get_id_from_name(msg->name[i]);
        if(id != -1)
        {
            syncwrite_addr = controller->getDevice(id)->ADDR_TORQUE_ENABLE;
            syncwrite_data_length = controller->getDevice(id)->getAddrLength(syncwrite_addr);
            syncwrite_param.resize(syncwrite_param.size() + syncwrite_data_length + 1); // 2 : ID(1) + TORQUE_ENABLE(1)
            syncwrite_param[n++]  = id;
            if(msg->enable[i] == true)
                syncwrite_param[n++]  = 1;
            else
                syncwrite_param[n++]  = 0;
        }
    }
    pthread_mutex_unlock(&mutex);
}

void *comm_thread_proc(void *param)
{
    ros::NodeHandle nh("~");

    ros::Publisher joint_states_pub;
    std::string topic_name;
    if(nh.getParam("publish_joint_topic_name", topic_name) == true)
        joint_states_pub = nh.advertise<sensor_msgs::JointState>(topic_name, 1);
    else
        joint_states_pub = nh.advertise<sensor_msgs::JointState>("/robot_joint_states", 1);

    ros::Publisher manager_ready_pub = nh.advertise<std_msgs::Bool>("/manager_ready", 10, true);

    int publish_rate = 125;
    if(nh.getParam("joint_state_publish_rate", publish_rate) == true)
    {
        if(publish_rate <= 0 || publish_rate > 125)
            publish_rate = 125;
    }
    else
    {
        publish_rate = 125;
    }
    ros::Rate control_rate(publish_rate);

    // check .launch file parameter
    if(controller->initialize() == false)
    {
        ROS_ERROR("robotis_controller initialize failed");
        return 0;
    }

    std_msgs::Bool ready;
    ready.data = true;
    manager_ready_pub.publish(ready);

    while(ros::ok())
    {
        // Run BulkRead
        grp_handler.runBulkRead();

        if(syncwrite_param.size() > 0) {
            pthread_mutex_lock(&mutex);
            int r = grp_handler.syncWrite(syncwrite_addr, syncwrite_data_length, &syncwrite_param[0], syncwrite_param.size());
            pthread_mutex_unlock(&mutex);
        }
        syncwrite_param.clear();

        // publish joint states
        sensor_msgs::JointState joint_states;
        if(publish_list.size() > 0)
        {
            for(int i = 0; i < publish_list.size(); i++)
            {
                int     _pos    = 0;
                int     _id     = publish_list[i];
                if(grp_handler.getReadData(_id, controller->getDevice(_id)->ADDR_PRESENT_POSITION, (long int*)&_pos) == true)
                {
                    joint_states.name.push_back(controller->getDevice(_id)->getJointName());
                    joint_states.position.push_back(controller->getDevice(_id)->value2Rad(_pos));


                }
            }
            joint_states.header.stamp = ros::Time::now();
            joint_states_pub.publish(joint_states);

//            for(int i = 0; i < publish_list.size(); i++)
//			{
//				int   _torque    = 0;
//
//				int     _id_2     = publish_list[i];
//				if(grp_handler.getReadCurrent(_id_2, controller->getDevice(_id_2)->ADDR_GOAL_TORQUE, (long int*)&_torque) == true)
//				{
//					ROS_ERROR("read_The_current");
//
//					ROS_ERROR("the current %i",_torque);
//
//				}
//				else{
//					ROS_ERROR("It is not possible to read the current");
//
//					ROS_ERROR("the port id %i", controller->getDevice(_id_2)->ADDR_GOAL_TORQUE);
//
//
//
//				}
//
//			}
//			ROS_ERROR("++++++++++++++++++++++++++++++++++");



        }


        ros::spinOnce();
        control_rate.sleep();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotis_manager");

    ros::NodeHandle nh("~");

    ros::Subscriber publish_position_sub    = nh.subscribe("/publish_position", 10, publish_position_callback);
    ros::Subscriber control_write_sub       = nh.subscribe("/control_write", 10, control_write_callback);
    ros::Subscriber control_torque_sub      = nh.subscribe("/control_torque", 10, control_torque_callback);

    cout<<"just test"<<endl;

    ros::Subscriber joint_states_sub;
    std::string topic_name;
    if(nh.getParam("subscribe_joint_topic_name", topic_name) == true)
        joint_states_sub = nh.subscribe(topic_name, 10, joint_states_callback);
    else
        joint_states_sub = nh.subscribe("/controller_joint_states", 10, joint_states_callback);

    pthread_t comm_thread;
    if(pthread_create(&comm_thread, 0, comm_thread_proc, 0) != 0)
        exit(-1);

    while(ros::ok())
    { }

    return 0;
}

