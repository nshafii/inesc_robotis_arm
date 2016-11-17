#ifndef cat_move_to_target_SIMPLE_TAG_HPP
#define cat_move_to_target_SIMPLE_TAG_HPP

/*-----------------------------------------*
 * Includes
 *-----------------------------------------*/
#include <geometry_msgs/PoseStamped.h>



/*-----------------------------------------*
 * Definition
 *-----------------------------------------*/


/*****************************************************************************
** Class [SIMPLE_TAG]
*****************************************************************************/
class SimpleTag{

private:
    int tag_id;
    geometry_msgs::PoseStamped pos;

public:

    SimpleTag();

    geometry_msgs::PoseStamped getPose();
    void setPose(geometry_msgs::PoseStamped);

    int getTagId();
    void setTagId(int id);

    std::vector<double> tagPositionDifference(SimpleTag &s);

    //falta o tagOrientationDifference
};

#endif
