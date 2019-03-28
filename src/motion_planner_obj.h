#ifndef MOTION_PLANNER_OBJ_H_
#define MOTION_PLANNER_OBJ_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "multi_agent_planner/agent_info.h"
#include "multi_agent_planner/get_plan.h"
#include <algorithm>
#include <climits>
#include <vector>
#include <cmath>

using std::vector;

struct Path
{
    std::string serial_id;
    vector<geometry_msgs::Point> point_list;
};

enum Status {FREE, OCCUPIED, START, GOAL};

struct Grid_node
{
    Status stat;
    bool is_closed;
    int past_cost;
    int total_cost;
    int pos[2];
    geometry_msgs::Point parent;

    bool operator<(Grid_node other) const
    {
        if (total_cost == other.total_cost)
        {
            if (pos[0] == other.pos[0])
            {
                return pos[1] < other.pos[1];
            }
            return pos[0] < other.pos[0];
        }
        return total_cost < other.total_cost;
    }
};

class Motion_Planner
{
public:
    explicit Motion_Planner(ros::NodeHandle *node_handle, const int X_max = 10, const int Y_max = 10, const int edge_cost = 10);
private:
    ros::NodeHandle node;
    ros::Publisher pub_grid_nodes_free;
    ros::Publisher pub_grid_nodes_occupied;
    ros::Subscriber sub_agent_pose;
    ros::ServiceServer srv_get_plan;

    const int X_MAX;
    const int Y_MAX;
    const int edge_cost;
    vector<Path> archived_paths;
    vector<multi_agent_planner::agent_info> agent_start_poses;

    void planner_draw_rviz_nodes();
    bool planner_get_plan(multi_agent_planner::get_plan::Request &req, multi_agent_planner::get_plan::Response &res);
    void planner_agent_pose_callback(multi_agent_planner::agent_info msg);
    vector<geometry_msgs::Point> planner_check_archives(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point);
    vector<geometry_msgs::Point> planner_plan_path(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point, const std::string serial_id);
};

#endif
