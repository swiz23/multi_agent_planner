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

// Path struct holds the serial_id of an agent along with the planned_path that it should follow
struct Path
{
    std::string serial_id;
    double time_of_plan;
    vector<geometry_msgs::Point> point_list;
};

// Used to describe the status of each node in the grid.
enum Status {FREE, OCCUPIED, START, GOAL};

// Node struct containing attributes that the A* algorithm will use. The comparison
// function described below deals with ties consistently so that the resulting
// planned path can be predicted and verified against the theoretical.
struct Grid_node
{
    Status stat;                                // Node status
    bool is_closed;                             // Has this node already been explored by the A* algorithm?
    int past_cost;                              // Cost from this node to the starting node
    int total_cost;                             // The 'past_cost' added to the cost needed to get to the Goal node
    int pos[2];                                 // Array holding the x,y position of the node in the grid
    geometry_msgs::Point parent;                // Point message holding x,y position of the parent node

    bool operator<(Grid_node other) const       // Comparison function used by A* to sort the nodes currently in the 'open' list
    {
        if (total_cost == other.total_cost)     // if the total cost of both nodes in 'open' are equal, then...
        {
            if (pos[0] == other.pos[0])         // if the x-position of the both nodes in 'open' are equal, then...
            {
                return pos[1] < other.pos[1];   // give precedence to the node with the lower 'y' position
            }
            return pos[0] < other.pos[0];       // else give precedence to the node with lower 'x' position
        }
        return total_cost < other.total_cost;   // else give precedence to the node with the lower cost
    }
};


class Motion_Planner
{
public:
    /// @brief Constructor for the Motion Planner
    explicit Motion_Planner(ros::NodeHandle *node_handle, const int X_max = 10, const int Y_max = 10, const int edge_cost = 10, const int period = 10);
private:
    ros::NodeHandle node;                                           // ROS node handler
    ros::Publisher pub_grid_nodes_free;                             // Publishes markers for each node that the agent can pass through
    ros::Publisher pub_grid_nodes_occupied;                         // Publishes markers for each node that the agent cannot pass through
    ros::Subscriber sub_agent_pose;                                 // Subscribes to /agent_feedback to get the current agent pose
    ros::ServiceServer srv_get_plan;                                // Service called to do the path planning

    const int X_MAX;                                                // Width of the grid - defaults to 10
    const int Y_MAX;                                                // Length of the grid - defaults to 10
    const int edge_cost;                                            // Cost to travel along all edges - defaults to 10
    const int period;                                               // Time in seconds for the agent to traverse the whole path - defaults to 10 seconds
    vector<Path> archived_paths;                                    // Holds the most up-to-date path of each agent
    vector<multi_agent_planner::agent_info> agent_start_poses;      // Holds the most up-to-date pose of each agent

    /// @brief Returns a vector containing the planned path for an agent - calculated using the A* algorithm
    /// @param start_point - where the algorithm should begin exploring
    /// @param goal_point - where the algorithm should stop exploring
    /// @param serial_id - name of the agent
    /// If no path is found, an empty vector is returned
    struct Path planner_plan_path(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point, const std::string serial_id, const vector<geometry_msgs::Point> collisions);

    /// @brief Returns node location of collision if there is one
    /// If there is no collision, then a point with a value of [X_MAX+1, Y_MAX+1] is returned
    /// @param p - freshly planned path to be checked against the other plans in the archives
    geometry_msgs::Point planner_check_collision(const struct Path current_path);

    /// @brief Service called to plan the path
    /// @param req - Service request containing the serial_id and goal_pose for an agent
    /// @param res - Serivce response containing the list of points represting the planned path
    bool planner_get_plan(multi_agent_planner::get_plan::Request &req, multi_agent_planner::get_plan::Response &res);

    /// @brief - Receives the newest pose of an agent and updates the 'agent_start_poses' vector with it
    /// @param msg - ROS message containing the serial_id and current pose of an agent
    void planner_agent_pose_callback(multi_agent_planner::agent_info msg);

    /// @brief - Publishes the markers representing the 'FREE' and 'OCCUPIED' nodes to Rviz
    void planner_draw_rviz_nodes();
};

#endif
