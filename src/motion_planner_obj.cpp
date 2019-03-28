#include "motion_planner_obj.h"

// @brief Constructor for the Motion Planner
Motion_Planner::Motion_Planner(ros::NodeHandle *node_handle, const int X_max, const int Y_max, const int edge_cost)
    : node(*node_handle), X_MAX(X_max), Y_MAX(Y_max), edge_cost(edge_cost)
{
    // Initialization of Publishers, Subscriber, and Service
    pub_grid_nodes_free = node.advertise<visualization_msgs::Marker>("visualization/grid_nodes_free", 100);
    pub_grid_nodes_occupied = node.advertise<visualization_msgs::Marker>("visualization/grid_nodes_occupied", 100);
    sub_agent_pose = node.subscribe("/agent_feedback", 100, &Motion_Planner::planner_agent_pose_callback, this);
    srv_get_plan = node.advertiseService("/get_plan", &Motion_Planner::planner_get_plan, this);
    ROS_INFO("Motion planner service (/get_plan) is ready.");

    // Draw the nodes in Rviz
    planner_draw_rviz_nodes();
}

/// @brief Returns a vector containing the planned path for an agent - calculated using the A* algorithm
/// @param start_point - where the algorithm should begin exploring
/// @param goal_point - where the algorithm should stop exploring
/// @param serial_id - name of the agent
/// If no path is found, an empty vector is returned
vector<geometry_msgs::Point> Motion_Planner::planner_plan_path(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point, const std::string serial_id)
{
    // Create empty vector 'open' where the A* algorithm will place nodes in the frontier
    vector<Grid_node> open;
    // 'final_path' will eventually hold the resulting path computed by the algorithm
    Path final_path {};
    final_path.serial_id = serial_id;

    // Create a 2-dimensional array of nodes
    Grid_node grid[X_MAX+1][Y_MAX+1] = {};
    for (size_t i{0}; i <= X_MAX; i++)
    {
        for (size_t j{0}; j <= Y_MAX; j++)
        {
            Grid_node n = {};
            // Optional - include an OCCUPIED region
            if ((i == 2 || i == 3) && (j == 2 || j ==3))
                n.stat = OCCUPIED;
            else
                n.stat = FREE;
            n.pos[0] = i;
            n.pos[1] = j;
            // Initialize the past_cost of all nodes except the first to a high value.
            // This ensures that a node's parent can be correctly determined.
            n.past_cost = INT_MAX;
            grid[i][j] = n;
        }
    }

    int start[] = {(int)start_point.x, (int)start_point.y};
    int goal[] = {(int)goal_point.x, (int)goal_point.y};
    grid[start[0]][start[1]].stat = START;
    grid[start[0]][start[1]].past_cost = 0;
    grid[goal[0]][goal[1]].stat = GOAL;
    open.push_back(grid[start[0]][start[1]]);

    // In general, each node has 4 neighbors - their relative positions to the
    // current node described in these two arrays
    int x_nbr_arr[] = {1, 0, -1, 0};
    int y_nbr_arr[] = {0, 1, 0, -1};
    int x_nbr{}, y_nbr{};

    int x_curr{}, y_curr{};
    Grid_node curr{};

    // Begin algorithm - adapted from Pg. 367 in "Modern Robotics"
    // by Kevin Lynch and Frank Park

    // While there are still nodes in the frontier...
    while (open.size() != 0)
    {
        // Look at first node
        curr = open.at(0);
        x_curr = curr.pos[0];
        y_curr = curr.pos[1];
        open.erase(open.begin());
        // Change its status to 'Closed'
        grid[x_curr][y_curr].is_closed = true;
        // if the node is the goal node...
        if (grid[x_curr][y_curr].stat == GOAL)
        {
            // add the goal node to the path list
            geometry_msgs::Point goal;
            goal.x = x_curr;
            goal.y = y_curr;
            final_path.point_list.push_back(goal);
            // Look at each node's parent and insert it to the beginning of the
            // list until the starting node is reached
            while (x_curr != start[0] || y_curr != start[1])
            {
                final_path.point_list.insert(final_path.point_list.begin(), grid[x_curr][y_curr].parent);
                x_curr = final_path.point_list.at(0).x;
                y_curr = final_path.point_list.at(0).y;
            }
            archived_paths.push_back(final_path);
            ROS_INFO("Algorithm found a path for %s", serial_id.c_str());
            break;
        }
        // Otherwise, look at the neighboring nodes...
        for (size_t i{0}; i < 4; i++)
        {
            x_nbr = x_curr + x_nbr_arr[i];
            y_nbr = y_curr + y_nbr_arr[i];
            // Don't fall off the grid!
            if (x_nbr >= 0 && x_nbr <= X_MAX && y_nbr >= 0 && y_nbr <= Y_MAX)
            {
                // Make sure the neighboring node has not aleady been explored and is not occupied
                if (!grid[x_nbr][y_nbr].is_closed && grid[x_nbr][y_nbr].stat != OCCUPIED)
                {
                    // Calculate the past_cost of the neighboring node assuming the current node is the parent
                    int tentative_past_cost = curr.past_cost + edge_cost;
                    // If the potential past_cost of the neighboring node is less than its current value, then assign
                    // the current node as its parent
                    if (tentative_past_cost < grid[x_nbr][y_nbr].past_cost)
                    {
                        grid[x_nbr][y_nbr].past_cost = tentative_past_cost;
                        grid[x_nbr][y_nbr].parent.x = x_curr;
                        grid[x_nbr][y_nbr].parent.y = y_curr;
                        // Calculate the total cost using Manhattan Distance as the 'Cost To Go' heuristic and add node to the frontier
                        grid[x_nbr][y_nbr].total_cost = grid[x_nbr][y_nbr].past_cost + edge_cost*(abs(x_nbr - goal[0]) + abs(y_nbr - goal[1]));
                        open.push_back(grid[x_nbr][y_nbr]);
                    }
                }
            }
        }
        // sort the list of nodes in the frontier based on cost using the custom comparator function
        // described in the 'Node' structure
        std::sort(open.begin(), open.end());
    }
    return final_path.point_list;
}

/// @brief Returns a vector containing part or all of a previously planned path
/// @param start_point - where the algorithm should begin exploring
/// @param goal_point - where the algorithm should stop exploring
/// If no path is found, an empty vector is returned
vector<geometry_msgs::Point> Motion_Planner::planner_check_archives(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point)
{
    // Empty vector to be returned if a path can not be found
    vector<geometry_msgs::Point> empty_vec;

    // 'start_pntr' will be assigned the address of the starting point if it is found in a previous path.
    // 'goal_pntr' will be assigned the address of the goal point if it is found in a previous path.
    geometry_msgs::Point *start_pntr {nullptr};
    geometry_msgs::Point *goal_pntr {nullptr};
    bool archived_start_found = false;
    bool archived_goal_found = false;

    // for each archived path...
    for (auto &path_obj : archived_paths)
    {
        // parse through each point in that path
        for (auto &point : path_obj.point_list)
        {
            // check to see if the start_point and archived point match. if they do,
            // assign the start_pntr to its address.
            if (point.x == start_point.x && point.y == start_point.y && !archived_start_found)
            {
                start_pntr = &point;
                archived_start_found = true;
            }
            // check to see if the goal_point and archived point match. if they do,
            // assign the goal_pntr to its address.
            if (point.x == goal_point.x && point.y == goal_point.y && !archived_goal_found)
            {
                goal_pntr = &point;
                archived_goal_found = true;
            }

            // if both points were found in the same path...
            if (archived_start_found && archived_goal_found)
            {
                ROS_INFO("Using an archived path originally created for %s", path_obj.serial_id.c_str());
                vector<geometry_msgs::Point>::iterator start_it(start_pntr);
                vector<geometry_msgs::Point>::iterator goal_it(goal_pntr);

                // if the 'start_pntr' has an address that is before 'goal_pntr' in memory...
                if (start_pntr < goal_pntr)
                {
                    // increment 'goal_it' to point at one past the goal point in memory
                    // so that the goal point is not cut out and create the sub array.
                    goal_it++;
                    vector<geometry_msgs::Point> sub_path(start_it, goal_it);
                    return sub_path;
                }
                // however, if the 'start_pntr' has an address that occurs after
                // the one in 'goal_pntr' in memory, we must flip the order.
                else
                {
                    start_it++;
                    vector<geometry_msgs::Point> sub_path(goal_it, start_it);
                    std::reverse(sub_path.begin(), sub_path.end());
                    return sub_path;
                }
            }
        }
        // if none or only one of the start/goal points were found in an archived path,
        // reassign the pointers to point at 0 so that they are ready for the next archived path.
        start_pntr = nullptr;
        goal_pntr = nullptr;
        archived_start_found = false;
        archived_goal_found = false;
    }
    return empty_vec;
}

/// @brief Service called to plan the path
/// @param req - Service request containing the serial_id and goal_pose for an agent
/// @param res - Serivce response containing the list of points represting the planned path
bool Motion_Planner::planner_get_plan(multi_agent_planner::get_plan::Request &req, multi_agent_planner::get_plan::Response &res)
{
    geometry_msgs::Point start_point, goal_point;
    goal_point.x = req.goal_pose.x;
    goal_point.y = req.goal_pose.y;

    // check the goal_point to make sure the user did not enter an invalid pose
    if (goal_point.x < 0 || goal_point.x > X_MAX || goal_point.y < 0 || goal_point.y > Y_MAX)
    {
        ROS_ERROR("That is an invalid goal pose.");
        return false;
    }

    // get the most up-to-date pose of the agent
    bool found = false;
    for (auto agent : agent_start_poses)
    {
        if (agent.serial_id == req.serial_id)
        {
            start_point.x = agent.start_pose.x;
            start_point.y = agent.start_pose.y;
            found = true;
            break;
        }
    }

    // if agent is not found, let the user know...
    if (!found)
    {
        ROS_ERROR("%s does not yet exist. Did you enter the wrong serial_id? - Exiting service...", req.serial_id.c_str());
        return false;
    }

    // check archived paths and return a path if one is found
    vector<geometry_msgs::Point> path;
    path = planner_check_archives(start_point, goal_point);
    if (path.size() != 0)
    {
        res.path = path;
        return true;
    }

    // If an archived path is not found, compute the path with the A* algorithm
    ROS_INFO("Using A* algorithm for %s as complete archived path could not be found", req.serial_id.c_str());
    path = planner_plan_path(start_point, goal_point, req.serial_id);
    if (path.size() != 0)
    {
        res.path = path;
        return true;
    }
    // The algorithm should not fail unless there are OCCUPIED nodes blocking all routes.
    ROS_WARN("Algorithm failed to find a path for %s", req.serial_id.c_str());
    return false;
}

/// @brief - Receives the newest pose of an agent and updates the 'agent_start_poses' vector with it
/// @param msg - ROS message containing the serial_id and current pose of an agent
void Motion_Planner::planner_agent_pose_callback(multi_agent_planner::agent_info msg)
{
    bool found = false;
    // The pose messages on the /agent_feedback topic can have 'non-integer'
    // values, so round to the nearest integer
    msg.start_pose.x = round(msg.start_pose.x);
    msg.start_pose.y = round(msg.start_pose.y);

    // if the agent is already in database, update its pose with the latest one.
    for (size_t i{0}; i < agent_start_poses.size(); i++)
    {
        if (agent_start_poses.at(i).serial_id == msg.serial_id)
        {
            agent_start_poses.at(i) = msg;
            found = true;
            break;
        }
    }
    // if the agent is not yet in databse, add it.
    if (!found)
    {
        agent_start_poses.push_back(msg);
    }
}

/// @brief - Publishes the markers representing the 'FREE' and 'OCCUPIED' nodes to Rviz
void Motion_Planner::planner_draw_rviz_nodes()
{
    visualization_msgs::Marker marker_free, marker_occupied;
    marker_free.header.frame_id = "/world";
    marker_free.header.stamp = ros::Time();
    marker_free.ns = "grid_nodes";
    marker_free.id = 0;
    marker_free.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_free.action = visualization_msgs::Marker::ADD;
    marker_free.pose.orientation.w = 1.0;
    marker_free.scale.x = 0.15;
    marker_free.scale.y = 0.15;
    marker_free.scale.z = 0.15;
    marker_free.color.r = 1.0;
    marker_free.color.b = 1.0;
    marker_free.color.a = 1.0;
    marker_free.lifetime = ros::Duration();

    marker_occupied = marker_free;
    marker_occupied.color.r = 1.0;
    marker_occupied.color.b = 0.0;
    marker_occupied.color.g = 1.0;
    marker_occupied.color.a = 1.0;

    for (size_t i {0}; i <= X_MAX; i++)
        for (size_t j{0}; j <= Y_MAX; j++)
        {
            geometry_msgs::Point p;
            p.x = i;
            p.y = j;
            // optional - draw occupied nodes as a different color
            if ((i == 2 || i == 3) && (j == 2 || j ==3))
                marker_occupied.points.push_back(p);
            else
                marker_free.points.push_back(p);
        }

    // don't publish the marker until Rviz is subscribed to it
    while (pub_grid_nodes_free.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         break;
       }
       sleep(1);
     }
    pub_grid_nodes_free.publish(marker_free);
    pub_grid_nodes_occupied.publish(marker_occupied);
}
