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
    /// @brief Constructor for the agent
    /// @param serial_id - the name of the agent
    explicit Agent_Robot(ros::NodeHandle *node_handle, std::string serial_id, geometry_msgs::Pose2D start_pose, const double period = 10.0, const double timer_hz = 30.0);
private:
    ros::NodeHandle node;                                   // ROS node handler
    ros::Publisher pub_agent_feedback;                      // Publishes current pose of agent
    ros::Publisher pub_agent_marker;                        // Publishes marker for the agent
    ros::Publisher pub_path_marker;                         // Publishes marker to represent the planned path
    ros::ServiceServer srv_update_goal;                     // Service that the user calls to update the goal pose for the agent
    ros::ServiceClient srv_get_plan;                        // Service that the node calls to get the planned path
    ros::Timer tmr_odom;                                    // Timer to continuously publish the agent pose, transform, and agent marker

    const double period;                                    // the time in seconds that it should take for the agent to traverse the path - defaults to 10
    const double timer_hz;                                  // the frequency at which the timer should run - defaults arbitrarily to 30 Hz
    double agent_color[3];                                  // Array to hold the RGB color of the agent
    double dt_orientation;                                  // increment used to update the orientation of the agent as it moves
    double dt_position;                                     // increment used to update the position of the agent as it moves
    bool rotate_only;                                       // When true, only the orientation of the agent is updated
    bool done;                                              // When true, the agent has reached its goal pose
    std::string serial_id;                                  // name of agent
    geometry_msgs::Pose2D pose;                             // current pose of the agent
    geometry_msgs::Pose2D goal_pose;                        // goal pose of the agent
    vector<geometry_msgs::Point> point_list;                // list of points that compose the path

    /// @brief Creates and publishes a LINE_STRIP marker representing the planned path
    /// @param vect - vector of points used to create the LINE_STRIP
    void agent_build_path_marker(const vector<geometry_msgs::Point> &vect);

    /// @brief Sends the transform of the agent with respect to the 'world' frame
    /// @param pose - the pose of the agent
    void agent_update_transform(const geometry_msgs::Pose2D &pose);

    /// @brief Timer function to continuously update the agent's pose as it travels a path
    /// @param e - Timer event message (not used)
    void agent_update_pose(const ros::TimerEvent &e);

    /// @brief Creates and publishes a CYLINDER marker representing the agent
    void agent_build_agent_marker();

    /// @brief Service called by the user to update the goal pose of the agent
    /// @param req - service request including the 2D pose of the agent
    /// @param res - service response containing the list of points representing the planned path (needed for Unit Test)
    bool agent_update_goal(multi_agent_planner::update_goal::Request &req, multi_agent_planner::update_goal::Response &res);

};

#endif
