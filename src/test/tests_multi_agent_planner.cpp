#include <ros/ros.h>
#include "multi_agent_planner/update_goal.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <gtest/gtest.h>
#include <vector>

TEST(PathPlanner, twoAgentsCanCollide)
{
    // setup ROS related service clients and node handle
    ros::NodeHandle node;
    ros::ServiceClient srv_update_goal_1 = node.serviceClient<multi_agent_planner::update_goal>("/agent_1/update_goal");
    ros::ServiceClient srv_update_goal_2 = node.serviceClient<multi_agent_planner::update_goal>("/agent_2/update_goal");

    // define the goal pose for each agent
    geometry_msgs::Pose2D goal_pose_1, goal_pose_2;
    goal_pose_1.x = 2;
    goal_pose_1.y = 5;
    goal_pose_1.theta = 0;

    goal_pose_2.x = 6;
    goal_pose_2.y = 3;
    goal_pose_2.theta = 0;

    // call the 'update_goal' service for both agents and retrieve their respective planned paths
    multi_agent_planner::update_goal srv_update_1, srv_update_2;
    srv_update_1.request.goal_pose = goal_pose_1;
    srv_update_2.request.goal_pose = goal_pose_2;
    // delay the service call for half a second to give time for the Motion Planner server to get the starting poses of each agent
    ros::Duration(0.5).sleep();
    srv_update_goal_1.call(srv_update_1);
    srv_update_goal_2.call(srv_update_2);
    std::vector<geometry_msgs::Point> actual_point_list_1, actual_point_list_2;
    actual_point_list_1 = srv_update_1.response.path;
    actual_point_list_2 = srv_update_2.response.path;

    // initialize two arrays with the expected paths for both agents
    double exp_point_list_1[6][2] = {{2,0}, {2,1}, {2,2}, {2,3}, {2,4}, {2,5}};
    double exp_point_list_2[9][2] = {{0,3}, {1,3}, {1,4}, {2,4}, {3,4}, {3,3}, {4,3}, {5,3}, {6,3}};

    // Test to make sure that every coordinate is correct for both paths
    for (size_t i{0}; i < 6; i++)
    {
        EXPECT_EQ(exp_point_list_1[i][0], actual_point_list_1.at(i).x);
        EXPECT_EQ(exp_point_list_1[i][1], actual_point_list_1.at(i).y);
    }

    for (size_t i{0}; i < 9; i++)
    {
        EXPECT_EQ(exp_point_list_2[i][0], actual_point_list_2.at(i).x);
        EXPECT_EQ(exp_point_list_2[i][1], actual_point_list_2.at(i).y);
    }

    // delay 10 seconds to give time for the agents to traverse their respective paths in Rviz
    ros::Duration(10).sleep();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tests_multi_agent_planner");
    return RUN_ALL_TESTS();
}
