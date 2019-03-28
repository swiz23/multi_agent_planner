#ifndef AGENT_OBJ_H_
#define AGENT_OBJ_H_

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Point.h>
#include "multi_agent_planner/update_goal.h"
#include "multi_agent_planner/agent_info.h"
#include "multi_agent_planner/get_plan.h"
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <ctime>


using std::vector;

const double pi = 3.1415926535897;

class Agent_Robot
{
public:
    explicit Agent_Robot(ros::NodeHandle *node_handle, std::string serial_id, geometry_msgs::Pose2D start_pose, const double period = 10.0, const double timer_hz = 30.0);
private:
    ros::NodeHandle node;
    ros::Publisher pub_agent_feedback;
    ros::Publisher pub_agent_marker;
    ros::Publisher pub_path_marker;
    ros::ServiceServer srv_update_goal;
    ros::ServiceClient srv_get_plan;
    ros::Timer tmr_odom;

    const double period;
    const double timer_hz;
    double agent_color[3];
    double dt_orientation;
    double dt_position;
    bool rotate_only;
    bool done;
    std::string serial_id;
    geometry_msgs::Pose2D pose;
    geometry_msgs::Pose2D goal_pose;
    vector<geometry_msgs::Point> point_list;

    void agent_build_path_marker(const vector<geometry_msgs::Point> &vect);
    void agent_update_transform(const geometry_msgs::Pose2D &pose);
    void agent_update_pose(const ros::TimerEvent &e);
    void agent_build_agent_marker();
    bool agent_update_goal(multi_agent_planner::update_goal::Request &req, multi_agent_planner::update_goal::Response &res);

};

#endif
