#include "motion_planner_obj.h"

// @brief Constructor for the Motion Planner
Motion_Planner::Motion_Planner(ros::NodeHandle *node_handle, const int X_max, const int Y_max, const int edge_cost, const int period)
    : node(*node_handle), X_MAX(X_max), Y_MAX(Y_max), edge_cost(edge_cost), period(period)
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
/// @param collisions - vector of nodes that should be treated as obstacles
/// If no path is found, an empty vector is returned
struct Path Motion_Planner::planner_plan_path(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point, const std::string serial_id, const vector<geometry_msgs::Point> collisions)
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
            // if ((i == 2 || i == 3) && (j == 2 || j == 3 || j == 4 || j == 5 || j == 6))
            //     n.stat = OCCUPIED;
            // else
            n.stat = FREE;
            n.pos[0] = i;
            n.pos[1] = j;
            // Initialize the past_cost of all nodes except the first to a high value.
            // This ensures that a node's parent can be correctly determined.
            n.past_cost = INT_MAX;
            grid[i][j] = n;
        }
    }

    // change status of nodes to OCCUPIED based on points in the 'collisions' vector
    for (auto &p : collisions)
    {
        grid[(int)p.x][(int)p.y].stat = OCCUPIED;
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
            final_path.time_of_plan = ros::Time::now().toSec();
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
    return final_path;
}

/// @brief Returns location of agent collision if there is one
/// If there is no collision, then a point with a value of [X_MAX+1, Y_MAX+1] is returned
/// @param current_path - freshly planned path to be checked against the other plans in the archives
geometry_msgs::Point Motion_Planner::planner_check_collision(const struct Path current_path)
{
    geometry_msgs::Point collision_point;
    collision_point.x = X_MAX + 1;
    collision_point.y = Y_MAX + 1;

    // Only the most recent path for each agent is saved in the archives. This is
    // all that is necessary to perform proper collision detection. So, for each
    // agent that has a stored path...
    for (auto &path_obj : archived_paths)
    {
        // make sure you are not comparing the agent to itself!
        if (path_obj.serial_id != current_path.serial_id)
        {
            // if more than 'period' seconds have past, that means the agent that
            // followed this path originally is already at the goal node.
            if ((current_path.time_of_plan - path_obj.time_of_plan) >= period)
            {
                // So all we need to do is to check to see if the goal node in the
                // archived path happens to be a point on the freshly planned path
                for (auto &current_path_point : current_path.point_list)
                {
                    // if that's the case, then we should treat this node as an obstacle
                    if (path_obj.point_list.back().x == current_path_point.x && path_obj.point_list.back().y == current_path_point.y)
                    {
                        return current_path_point;
                    }
                }
            }
            // however, if fewer than 'period' seconds have passed, we need
            // to test to see if the current agent will collide with the previous
            // agent somewhere along the path. To do this, we need to find where
            // every point in the currently planned path intersects with a point in
            // an archived path.
            else
            {
                // for each archived path...
                for (size_t i{0}; i < path_obj.point_list.size(); i++)
                {
                    // for each point in the current path...
                    for (size_t j{0}; j < current_path.point_list.size(); j++)
                    {
                        // if a point in the currently planned path intersects with one in an archived path...
                        if (path_obj.point_list.at(i).x == current_path.point_list.at(j).x && path_obj.point_list.at(i).y == current_path.point_list.at(j).y)
                        {
                            // Now, we need to see if the intersection is a problem. It could very well be that the previous agent will have already passed the
                            // intersection before this agent will cross it - meaning that the planned path would be fine to use.

                            // first, let's figure out how much time has passed since the previous agent started traversing its own path.
                            double time_offset_archived_path = current_path.time_of_plan - path_obj.time_of_plan;

                            // second, determine the number of edges in the currently planned path
                            double num_segs_current_path = current_path.point_list.size() - 1;

                            // third, figure out how long it takes (in seconds) for the current agent to traverse one edge assuming it should take 'period' seconds to complete the whole path
                            double sec_per_seg_current_path = period/num_segs_current_path;

                            // repeat these steps for the agent in the archived path
                            double num_segs_archived_path = path_obj.point_list.size() - 1;
                            double sec_per_seg_archived_path = period/num_segs_archived_path;

                            // since each agent is 1 meter in diameter, it will take the time necessary to travel two edges for the agent to be out of the 'danger' zone. So, calculate at what time
                            // the current agent will enter the 'danger' zone and when it will be safely across. Time '0' is when the current agent begins to move
                            double time_before_collision_current_path = (j - 1) * sec_per_seg_current_path;
                            double time_after_collision_current_path = (j + 1) * sec_per_seg_current_path;

                            // now calculate when the 'past' agent will arrive at this intersection. Make sure to subtract the difference in time between when the path was planned for the 'past'
                            // agent and for the current agent.
                            double time_before_collision_archived_path = (i - 1) * sec_per_seg_archived_path - time_offset_archived_path;
                            double time_after_collision_archived_path = (i + 1) * sec_per_seg_archived_path - time_offset_archived_path;

                            // Check for the inverse of the following...
                            // if the current agent passes through the intersection before the 'past' agent starts to cross OR
                            // if the current agent passes through the intersection after the 'past' agent has already crossed
                            if (!(time_after_collision_current_path <= time_before_collision_archived_path || time_before_collision_current_path >= time_after_collision_archived_path))
                            {
                                // then there is a collision, and we must treat this point as an obstacle.
                                return current_path.point_list.at(j);
                            }
                        }
                    }
                }
            }
        }
    }

    return collision_point;
}

/// @brief Service called to plan the path
/// @param req - Service request containing the serial_id and goal_pose for an agent
/// @param res - Serivce response containing the list of points representing the planned path
bool Motion_Planner::planner_get_plan(multi_agent_planner::get_plan::Request &req, multi_agent_planner::get_plan::Response &res)
{
    // round the goal input to the nearest integer just in case the user entered a 'non-integer' number
    geometry_msgs::Point start_point, goal_point;
    goal_point.x = round(req.goal_pose.x);
    goal_point.y = round(req.goal_pose.y);

    // check the goal_point to make sure the user did not enter an invalid pose
    if (goal_point.x < 0 || goal_point.x > X_MAX || goal_point.y < 0 || goal_point.y > Y_MAX)
    {
        ROS_ERROR("That is an invalid goal pose.");
        return false;
    }

    // check to make sure that a different agent is not already occupying the requested goal point
    for (auto &path_obj : archived_paths)
    {
        if (path_obj.serial_id != req.serial_id)
        {
            if (path_obj.point_list.back().x == goal_point.x && path_obj.point_list.back().y == goal_point.y)
            {
                ROS_ERROR("You entered a goal point that will be or currently is occupied by %s. Please choose a different goal.", path_obj.serial_id.c_str());
                return false;
            }
        }
    }

    // get the most up-to-date pose of the agent
    bool found = false;
    for (auto &agent : agent_start_poses)
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

    vector<geometry_msgs::Point> collisions;
    geometry_msgs::Point collision_location;
    Path current_path {};

    ROS_INFO("Using A* algorithm for %s.", req.serial_id.c_str());

    do
    {
        // compute the path with the A* algorithm
        current_path = planner_plan_path(start_point, goal_point, req.serial_id, collisions);
        // check to see if there is a potential collision. if so, return the location of it
        collision_location = planner_check_collision(current_path);
        // add the location to a vector. as collision locations increase, this vector will increase
        collisions.push_back(collision_location);
        // keep iterating until a collision-free path is found
    } while(collision_location.x != (X_MAX + 1) && collision_location.y != (Y_MAX + 1) && current_path.point_list.size() != 0);

    // make sure that the algorithm actually found a path
    if (current_path.point_list.size() != 0)
    {
        ROS_INFO("Algorithm found a path for %s.", req.serial_id.c_str());
        res.path = current_path.point_list;
        found = false;
        // update the path in the archives for this agent or tack it on to the end
        // if this is the agent's first path. Note that technically, an agent's first
        // 'path' is defined as its starting position. So, we should never have to
        // 'push_back' a new path. I just included it for completeness.
        for (auto &path_obj : archived_paths)
        {
            if (path_obj.serial_id == current_path.serial_id)
            {
                path_obj.time_of_plan = current_path.time_of_plan;
                path_obj.point_list = current_path.point_list;
                found = true;
            }
        }
        if (!found)
        {
            archived_paths.push_back(current_path);
        }
        return true;
    }
    // The algorithm should not fail unless there are OCCUPIED nodes blocking all routes.
    ROS_WARN("Algorithm failed to find a path for %s.", req.serial_id.c_str());
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
    // if the agent is not yet in databse, add it. Also, initialize a Path struct
    // and add the agent with its starting point into the archives.
    if (!found)
    {
        agent_start_poses.push_back(msg);
        Path new_agent_path {};
        geometry_msgs::Point new_point;
        new_agent_path.serial_id = msg.serial_id;
        new_agent_path.time_of_plan = ros::Time::now().toSec();
        new_point.x = msg.start_pose.x;
        new_point.y = msg.start_pose.y;
        new_agent_path.point_list.push_back(new_point);
        archived_paths.push_back(new_agent_path);
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
            // if ((i == 2 || i == 3) && (j == 2 || j == 3 || j == 4 || j == 5 || j == 6))
            //     marker_occupied.points.push_back(p);
            // else
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
