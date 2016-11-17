#include "../include/cat_move_to_target/SimpleTag.hpp"

SimpleTag::SimpleTag(){
    this->tag_id= -1000;

    this->pos.pose.position.x = -1000;
    this->pos.pose.position.y = -1000;
    this->pos.pose.position.z = -1000;

    this->pos.pose.orientation.x = -1000;
    this->pos.pose.orientation.y = -1000;
    this->pos.pose.orientation.z = -1000;
    this->pos.pose.orientation.w = -1000;
}

geometry_msgs::PoseStamped SimpleTag::getPose(){
    return this->pos;
}

void SimpleTag::setPose(geometry_msgs::PoseStamped p){
    this->pos = p;
}

int SimpleTag::getTagId(){
    return this->tag_id;
}

void SimpleTag::setTagId(int id){
    this->tag_id = id;
}

std::vector<double> SimpleTag::tagPositionDifference(SimpleTag &s){

    geometry_msgs::PoseStamped s_pos = s.getPose();

    double x_diff = s_pos.pose.position.x - (this->pos).pose.position.x;
    double y_diff = s_pos.pose.position.y - (this->pos).pose.position.y;
    double z_diff = s_pos.pose.position.z - (this->pos).pose.position.z;

    std::vector<double> position_difference;
    position_difference.push_back(x_diff);
    position_difference.push_back(y_diff);
    position_difference.push_back(z_diff);

    return position_difference;
}
