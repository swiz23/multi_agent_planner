#include "motion_planner_obj.h"

Motion_Planner::Motion_Planner(ros::NodeHandle *node_handle, const int X_max, const int Y_max, const int edge_cost)
    : node(*node_handle), X_MAX(X_max), Y_MAX(Y_max), edge_cost(edge_cost)
{
    pub_grid_nodes_free = node.advertise<visualization_msgs::Marker>("visualization/grid_nodes_free", 100);
    pub_grid_nodes_occupied = node.advertise<visualization_msgs::Marker>("visualization/grid_nodes_occupied", 100);
    sub_agent_pose = node.subscribe("/agent_feedback", 100, &Motion_Planner::planner_agent_pose_callback, this);
    srv_get_plan = node.advertiseService("/get_plan", &Motion_Planner::planner_get_plan, this);
    ROS_INFO("Motion planner service (/get_plan) is ready.");

    planner_draw_rviz_nodes();
}

vector<geometry_msgs::Point> Motion_Planner::planner_check_archives(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point)
{
    vector<geometry_msgs::Point> empty_vec;
    geometry_msgs::Point *start_pntr {nullptr};
    geometry_msgs::Point *goal_pntr {nullptr};
    bool archived_start_found = false;
    bool archived_goal_found = false;
    for (auto &path_obj : archived_paths)
    {
        for (auto &point : path_obj.point_list)
        {
            if (point.x == start_point.x && point.y == start_point.y && !archived_start_found)
            {
                start_pntr = &point;
                archived_start_found = true;
            }
            if (point.x == goal_point.x && point.y == goal_point.y && !archived_goal_found)
            {
                goal_pntr = &point;
                archived_goal_found = true;
            }
            if (archived_start_found && archived_goal_found)
            {
                ROS_INFO("Using an archived path originally created for %s", path_obj.serial_id.c_str());
                vector<geometry_msgs::Point>::iterator start_it(start_pntr);
                vector<geometry_msgs::Point>::iterator goal_it(goal_pntr);
                if (start_pntr < goal_pntr)
                {
                    goal_it++;
                    vector<geometry_msgs::Point> sub_path(start_it, goal_it);
                    return sub_path;
                }
                else
                {
                    start_it++;
                    vector<geometry_msgs::Point> sub_path(goal_it, start_it);
                    std::reverse(sub_path.begin(), sub_path.end());
                    return sub_path;
                }
            }
        }
        start_pntr = nullptr;
        goal_pntr = nullptr;
        archived_start_found = false;
        archived_goal_found = false;
    }
    return empty_vec;
}

vector<geometry_msgs::Point> Motion_Planner::planner_plan_path(const geometry_msgs::Point start_point, const geometry_msgs::Point goal_point, const std::string serial_id)
{
    // A* algorithm
    vector<Grid_node> open;
    Path final_path {};
    final_path.serial_id = serial_id;
    Grid_node grid[X_MAX+1][Y_MAX+1] = {};
    for (size_t i{0}; i <= X_MAX; i++)
    {
        for (size_t j{0}; j <= Y_MAX; j++)
        {
            Grid_node n = {};
            if ((i == 2 || i == 3) && (j == 2 || j ==3))
                n.stat = OCCUPIED;
            else
                n.stat = FREE;
            n.pos[0] = i;
            n.pos[1] = j;
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

    int x_nbr_arr[] = {1, 0, -1, 0};
    int y_nbr_arr[] = {0, 1, 0, -1};
    int x_nbr{}, y_nbr{};

    int x_curr{}, y_curr{};
    Grid_node curr{};

    while (open.size() != 0)
    {
        curr = open.at(0);
        x_curr = curr.pos[0];
        y_curr = curr.pos[1];
        open.erase(open.begin());
        grid[x_curr][y_curr].is_closed = true;
        if (grid[x_curr][y_curr].stat == GOAL)
        {
            geometry_msgs::Point goal;
            goal.x = x_curr;
            goal.y = y_curr;
            final_path.point_list.push_back(goal);
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
        for (size_t i{0}; i < 4; i++)
        {
            x_nbr = x_curr + x_nbr_arr[i];
            y_nbr = y_curr + y_nbr_arr[i];
            if (x_nbr >= 0 && x_nbr <= X_MAX && y_nbr >= 0 && y_nbr <= Y_MAX)
            {
                if (!grid[x_nbr][y_nbr].is_closed && grid[x_nbr][y_nbr].stat != OCCUPIED)
                {
                    int tentative_past_cost = curr.past_cost + edge_cost;
                    if (tentative_past_cost < grid[x_nbr][y_nbr].past_cost)
                    {
                        grid[x_nbr][y_nbr].past_cost = tentative_past_cost;
                        grid[x_nbr][y_nbr].parent.x = x_curr;
                        grid[x_nbr][y_nbr].parent.y = y_curr;
                        grid[x_nbr][y_nbr].total_cost = grid[x_nbr][y_nbr].past_cost + edge_cost*(abs(x_nbr - goal[0]) + abs(y_nbr - goal[1]));
                        open.push_back(grid[x_nbr][y_nbr]);
                    }
                }
            }
        }
        std::sort(open.begin(), open.end());
    }
    return final_path.point_list;
}

bool Motion_Planner::planner_get_plan(multi_agent_planner::get_plan::Request &req, multi_agent_planner::get_plan::Response &res)
{
    geometry_msgs::Point start_point, goal_point;
    goal_point.x = req.goal_pose.x;
    goal_point.y = req.goal_pose.y;

    if (goal_point.x < 0 || goal_point.x > X_MAX || goal_point.y < 0 || goal_point.y > Y_MAX)
    {
        ROS_ERROR("That is an invalid goal pose.");
        return false;
    }

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

    if (!found)
    {
        ROS_ERROR("%s does not yet exist - Exiting service...", req.serial_id.c_str());
        return false;
    }

    vector<geometry_msgs::Point> path;
    path = planner_check_archives(start_point, goal_point);
    if (path.size() != 0)
    {
        res.path = path;
        return true;
    }

    ROS_INFO("Using A* algorithm for %s as complete archived path could not be found", req.serial_id.c_str());
    path = planner_plan_path(start_point, goal_point, req.serial_id);
    if (path.size() != 0)
    {
        res.path = path;
        return true;
    }
    ROS_WARN("Algorithm failed to find a path for %s", req.serial_id.c_str());
    return false;
}

void Motion_Planner::planner_agent_pose_callback(multi_agent_planner::agent_info msg)
{
    bool found = false;
    msg.start_pose.x = round(msg.start_pose.x);
    msg.start_pose.y = round(msg.start_pose.y);

    for (size_t i{0}; i < agent_start_poses.size(); i++)
    {
        if (agent_start_poses.at(i).serial_id == msg.serial_id)
        {
            agent_start_poses.at(i) = msg;
            found = true;
            break;
        }
    }
    if (!found)
    {
        agent_start_poses.push_back(msg);
    }
}

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
            if ((i == 2 || i == 3) && (j == 2 || j ==3))
                marker_occupied.points.push_back(p);
            else
                marker_free.points.push_back(p);
        }
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
