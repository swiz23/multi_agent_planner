#include "agent_obj.h"


// Constructor for the agent
Agent_Robot::Agent_Robot(ros::NodeHandle *node_handle, std::string serial_id, geometry_msgs::Pose2D start_pose, const double period, const double timer_hz)
    : node(*node_handle), serial_id(serial_id), pose(start_pose), period(period), timer_hz(timer_hz)
{
    // Seed the random number generator using the number at the end of the agent's serial_id.
    // The random numbers are used to create a unique marker color for each agent
    int a = serial_id.at(6);
    srand (time(NULL)+ a);
    agent_color[0] = (rand() % 100)*0.01;
    agent_color[1] = (rand() % 100)*0.01;
    agent_color[2] = (rand() % 100)*0.01;
    dt_orientation = 0;
    dt_position = 0;
    rotate_only = false;
    done = true;

    // Publishers, Services, and Timer initializations
    pub_agent_marker = node.advertise<visualization_msgs::Marker>("visualization/base_link", 100);
    pub_path_marker = node.advertise<visualization_msgs::Marker>("visuallization/path", 100);
    pub_agent_feedback = node.advertise<multi_agent_planner::agent_info>("/agent_feedback", 100);
    srv_update_goal = node.advertiseService("update_goal", &Agent_Robot::agent_update_goal, this);
    srv_get_plan = node.serviceClient<multi_agent_planner::get_plan>("/get_plan");
    tmr_odom = node.createTimer(ros::Duration(1/timer_hz), &Agent_Robot::agent_update_pose, this);
    ROS_INFO("Ready to update goal pose for %s.", serial_id.c_str());
}

/// @brief Creates and publishes a LINE_STRIP marker representing the planned path
/// @param vect - vector of points used to create the LINE_STRIP
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
    marker.scale.x = 0.05;
    marker.color.r = agent_color[0];
    marker.color.g = agent_color[1];
    marker.color.b = agent_color[2];
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker.points = vect;
    pub_path_marker.publish(marker);
}

/// @brief Sends the transform of the agent with respect to the 'world' frame
/// @param pose - the pose of the agent
void Agent_Robot::agent_update_transform(const geometry_msgs::Pose2D &pose)
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

/// @brief Timer function to continuously update the agent's pose as it travels a path
/// @param e - Timer event message (not used)
void Agent_Robot::agent_update_pose(const ros::TimerEvent &e)
{
    static int cntr{0}, rotate_cntr{0}, index {0};
    static double num_cycles_in_period = period * timer_hz;

    // if the agent is not yet done traversing the planned path...
    if (!done)
    {
        // if the agent is translating in addition to rotating
        if (!rotate_only)
        {
            // increment the x and y positions of the agent along the path
            // ex. if the agent is traveling from (1,1) to (1,2), then 'pose.x'
            // will remain the same, but pose.y will increment by 'dt_position.'
            pose.x += (point_list.at(index + 1).x - point_list.at(index).x)*dt_position;
            pose.y += (point_list.at(index + 1).y - point_list.at(index).y)*dt_position;

            // it is a distance of 1 meter to get from one node to the next so
            // after enough Timer iterations, 'pose.x' and/or 'pose.y' will have
            // been incremented by this amount. If that's the case...
            if (cntr*dt_position >= 1)
            {
                // reset the counter and increment the index so that the next
                // node on the path will be considered.
                cntr = 0;
                index++;
                // to reduce the error induced by floating point calculations,
                // reset the pose of the agent at each node to the node's position.
                pose.x = point_list.at(index).x;
                pose.y = point_list.at(index).y;
            }
            // increment the counter used to keep track of the agent's position.
            cntr++;
        }
        // increment the orientation of the agent in a similar manner - the orientation
        // is incremented over the course of the whole path.
        pose.theta += dt_orientation;

        // Theoretically, assuming a timer frequency of 30 Hz and a travel time of 10
        // seconds, it should take 30*10 = 300 iterations (i.e. 'num_cycles_in_period')
        // for the agent to reach its goal. If the agent is undergoing only pure rotation,
        // then this is the metric used to determine if the orientation goal was achieved.
        // However, possibly due to floating point error during the pose updates, the goal
        // pose of the agent may or may not have been achieved yet. As such, keep updating
        // the linear pose of the agent until all nodes in the path (point_list) have been
        // processed - this usually means iterating a couple more times.

        if ((rotate_cntr >= num_cycles_in_period && rotate_only) || (!rotate_only && index == point_list.size()-1))
        {
            pose.theta = goal_pose.theta;
            done = true;
            index = 0;
            rotate_only = false;
            rotate_cntr = 0;
            ROS_INFO("Target goal has been reached by %s.", serial_id.c_str());
        }
        rotate_cntr++;
    }

    // Publish the updated pose to /agent_feedback, send out the transform, and
    // publish an agent marker.
    multi_agent_planner::agent_info msg;
    msg.serial_id = serial_id;
    msg.start_pose = pose;
    pub_agent_feedback.publish(msg);
    agent_update_transform(pose);
    agent_build_agent_marker();
}

/// @brief Creates and publishes a CYLINDER marker representing the agent
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

/// @brief Service called by the user to update the goal pose of the agent
/// @param req - service request including the 2D pose of the agent
/// @param res - service response containing the list of points representing the planned path (needed for Unit Test)
bool Agent_Robot::agent_update_goal(multi_agent_planner::update_goal::Request &req, multi_agent_planner::update_goal::Response &res)
{
    // Some Input Validation...
    if (!done)
    {
        ROS_INFO("Please wait until %s reaches its target destination to set another path.", serial_id.c_str());
        return false;
    }

    goal_pose = req.goal_pose;
    goal_pose.theta *=  pi / 180.0;
    if (goal_pose.x == pose.x && goal_pose.y == pose.y && goal_pose.theta == pose.theta)
    {
        ROS_INFO("%s is already at the goal! Please choose a different goal position.", serial_id.c_str());
        return false;
    }

    // Check to see if the goal_pose is the same as the start_pose except for the orientation.
    // If that's the case, do not call the /get_plan service. Instead, just rotate the agent.
    else if (goal_pose.x == pose.x && goal_pose.y == pose.y && goal_pose.theta != pose.theta)
    {
        // Find the increment needed to update the orientation such that it takes 'period' seconds.
        // Assuming a period of 10 seconds and a Timer frequency of 30 Hz, this value would be equal
        // to the difference in orientation divided by 300.
        dt_orientation = ((goal_pose.theta - pose.theta)/period)/timer_hz;
        ROS_INFO("%s is rotating...", serial_id.c_str());
        rotate_only = true;
        done = false;
    }
    else
    {
        // Call the /get_plan service
        multi_agent_planner::get_plan srv;
        srv.request.serial_id = serial_id;
        srv.request.goal_pose = goal_pose;
        bool success = srv_get_plan.call(srv);
        // if a plan was found (which should always be the case)...
        if (success)
        {
            // Calculate the increment needed to update the pose of the agent so
            // it takes 'period' seconds to translate uniformly. To do this, find
            // the number of edges to traverse and divide by the 'period' and
            // Timer frequency. Ex - if the Timer frequency is 30 Hz, the number
            // of edges in the path is 8, and it should take 10 seconds for the
            // agent to traverse the path, then it should take 8/10 = 0.8 seconds
            // to traverse one edge. Divide by 30 to account for the timer and the
            // result is 0.02667 meters/cycle.
            point_list = srv.response.path;
            res.path = point_list;
            int segments{};
            segments = point_list.size() - 1;
            dt_position = (segments/period)/timer_hz;
            dt_orientation = ((goal_pose.theta - pose.theta)/period)/timer_hz;
            agent_build_path_marker(point_list);
            done = false;
        }
    }
    return true;
}
