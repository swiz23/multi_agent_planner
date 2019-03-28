#include "agent_obj.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "agent");
    ros::NodeHandle n;

    // Describe usage and check for proper args
    static const char USAGE[] =
    "Usage: agent_node SERIAL_ID X Y YAW\n"
    "SERIAL_ID - the name of the robot. Used for ROS topic namespaces and robot identification.\n"
    "X - The 'x' coordinate of the robot's starting position (in meters).\n"
    "Y - The 'y' coordinate of the robot's starting position (in meters).\n"
    "YAW - The 'yaw' coordinate of the robot's starting position (in degrees).\n";

    if (argc != 5)
    {
        ROS_INFO(USAGE);
        exit(EXIT_FAILURE);
    }
    geometry_msgs::Pose2D start_pose;
    start_pose.x = atof(argv[2]);
    start_pose.y = atof(argv[3]);
    start_pose.theta = atof(argv[4]) * pi / 180.0;

    // Create instance of Agent_Robot
    Agent_Robot agent(&n, argv[1], start_pose);
    ros::spin();
    return 0;
}
