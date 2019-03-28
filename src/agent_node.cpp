#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "multi_agent_planner/get_plan.h"
#include "multi_agent_planner/update_goal.h"
#include "multi_agent_planner/agent_info.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::vector;

const double pi = 3.1415926535897;

class Agent_Robot
{
public:
    Agent_Robot(ros::NodeHandle *node_handle, std::string serial_id, double x, double y, double yaw);
private:
    ros::NodeHandle node;
    ros::Publisher pub_agent_marker;
    ros::Publisher pub_path_marker;
    ros::Publisher pub_agent_feedback;
    ros::ServiceServer srv_update_goal;
    ros::ServiceClient srv_get_plan;
    ros::Timer tmr_odom;
    double x_start;
    double y_start;
    double yaw_start;
    int id;
    double agent_color[3];
    bool done;
    bool rotate_only;
    const double period {10.0};
    const double timer_hz {30.0};
    double dt_position;
    double dt_orientation;
    std::string serial_id;
    geometry_msgs::Pose2D pose;
    geometry_msgs::Pose2D goal_pose;
    vector<geometry_msgs::Point> point_list;
    void agent_update_transform(geometry_msgs::Pose2D &pose);
    void agent_build_path_marker(const vector<geometry_msgs::Point> &vect);
    void agent_build_agent_marker();
    void agent_update_pose(const ros::TimerEvent &e);
    bool agent_update_goal(multi_agent_planner::update_goal::Request &req, multi_agent_planner::update_goal::Response &res);

};

Agent_Robot::Agent_Robot(ros::NodeHandle *node_handle, std::string serial_id, double x, double y, double yaw)
    : node(*node_handle), serial_id(serial_id), x_start(x), y_start(y), yaw_start(yaw)
{
    char a = serial_id.at(6);
    id = a - '0';
    agent_color[0] = 0.0;
    agent_color[1] = 0.0;
    agent_color[2] = 0.0;
    agent_color[id-1] = 1.0;
    pose.x = x_start;
    pose.y = y_start;
    pose.theta = yaw_start * pi / 180.0;
    done = true;
    dt_position = 0;
    dt_orientation = 0;
    rotate_only = false;

    pub_agent_marker = node.advertise<visualization_msgs::Marker>("visualization/base_link", 100);
    pub_path_marker = node.advertise<visualization_msgs::Marker>("visuallization/path", 100);
    pub_agent_feedback = node.advertise<multi_agent_planner::agent_info>("/agent_feedback", 100);
    srv_update_goal = node.advertiseService("update_goal", &Agent_Robot::agent_update_goal, this);
    srv_get_plan = node.serviceClient<multi_agent_planner::get_plan>("/get_plan");
    tmr_odom = node.createTimer(ros::Duration(1/timer_hz), &Agent_Robot::agent_update_pose, this);
    ROS_INFO("Ready to update goal position for %s.", serial_id.c_str());
}

void Agent_Robot::agent_update_pose(const ros::TimerEvent &e)
{
    static int cntr{0}, rotate_cntr{0}, index {0};
    if (!done && !rotate_only)
    {
        pose.x += (point_list.at(index + 1).x - point_list.at(index).x)*dt_position;
        pose.y += (point_list.at(index + 1).y - point_list.at(index).y)*dt_position;
        if (rotate_cntr < period * timer_hz)
            pose.theta += dt_orientation;
        cntr++;
        rotate_cntr++;
        if (cntr*dt_position >= 1)
        {
            cntr = 0;
            index++;
            pose.x = point_list.at(index).x;
            pose.y = point_list.at(index).y;
            if (index == point_list.size()-1)
            {
                pose.theta = goal_pose.theta;
                done = true;
                index = 0;
                rotate_cntr = 0;
                ROS_INFO("Target goal has been reached by %s.", serial_id.c_str());
            }
        }
    }
    else if (!done && rotate_only)
    {
        pose.theta += dt_orientation;
        rotate_cntr++;
        if (rotate_cntr >= period * timer_hz)
        {
            pose.theta = goal_pose.theta;
            done = true;
            rotate_only = false;
            rotate_cntr = 0;
            ROS_INFO("Rotation goal has been reached by %s.", serial_id.c_str());
        }
    }

    multi_agent_planner::agent_info msg;
    msg.serial_id = serial_id;
    msg.start_pose = pose;
    pub_agent_feedback.publish(msg);
    agent_update_transform(pose);
    agent_build_agent_marker();
}

bool Agent_Robot::agent_update_goal(multi_agent_planner::update_goal::Request &req, multi_agent_planner::update_goal::Response &res)
{
    if (!done)
    {
        ROS_INFO("Please wait until %s reaches its target destination to set another path.", serial_id.c_str());
        return false;
    }

    goal_pose = req.goal_pose;
    goal_pose.theta *=  pi / 180.0;
    if (goal_pose.x == pose.x && goal_pose.y == pose.y && goal_pose.theta == pose.theta)
    {
        ROS_INFO("The agent is already at the goal! Please choose a different goal position.");
        return false;
    }
    else if (goal_pose.x == pose.x && goal_pose.y == pose.y && goal_pose.theta != pose.theta)
    {
        dt_orientation = ((goal_pose.theta - pose.theta)/period)/timer_hz;
        ROS_INFO("%s is rotating...", serial_id.c_str());
        rotate_only = true;
        done = false;
    }
    else
    {
        multi_agent_planner::get_plan srv;
        srv.request.serial_id = serial_id;
        srv.request.goal_pose = goal_pose;
        srv_get_plan.call(srv);
        point_list = srv.response.path;
        int segments{};
        segments = point_list.size() - 1;
        dt_position = (segments/period)/timer_hz;
        yaw_start = pose.theta;
        dt_orientation = ((goal_pose.theta - pose.theta)/period)/timer_hz;
        agent_build_path_marker(point_list);
        done = false;
    }
    return true;
}

void Agent_Robot::agent_build_path_marker(const vector<geometry_msgs::Point> &vect)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.ns = serial_id;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.color.r = agent_color[0];
    marker.color.g = agent_color[1];
    marker.color.b = agent_color[2];
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker.points = vect;

    pub_path_marker.publish(marker);
}

void Agent_Robot::agent_build_agent_marker()
{
    // Build marker message for the robot
    visualization_msgs::Marker marker;
    marker.header.frame_id = serial_id + "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = serial_id;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0.25;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = agent_color[0];
    marker.color.g = agent_color[1];
    marker.color.b = agent_color[2];

    pub_agent_marker.publish(marker);
}

void Agent_Robot::agent_update_transform(geometry_msgs::Pose2D &pose)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);
    q.normalize();
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "/world";
    transform.child_frame_id = serial_id + "/base_link";
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    br.sendTransform(transform);
}

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

    Agent_Robot agent(&n, argv[1], atof(argv[2]), atof(argv[3]), atof(argv[4]));
    ros::spin();
    return 0;
}
