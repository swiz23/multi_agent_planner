#include "motion_planner_obj.h"

int main( int argc, char** argv )
{
    // Create instance of Motion_planner
    ros::init(argc, argv, "nodes");
    ros::NodeHandle n;
    Motion_Planner mp(&n);
    ros::spin();
    return 0;
}
