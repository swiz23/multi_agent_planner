# Multi-agent Path Planner
Created by Solomon Wiznitzer
## Description
**Objective:** To perform motion planning for at least 2 robots (where each robot can only move along the edges between nodes) and display the planned paths in Rviz.

**How It Works:** There were four parts to this project.
- *Creating a Roadmap* - This part included configuring Rviz to display a 10x10 grid with (0,0) being at the bottom left corner and (10,10) being at the top right corner. It also entailed drawing a marker representation of all the nodes.
- *Coding the Planner node* - As its name suggests, this involved implementing an algorithm to determine the minimum distance path from a robot's initial starting position to a user-specified goal position. Additionally, the node was responsible for archiving the planned paths so that they could potentially be reused for future robots.
- *Coding the Agent node* - The third part required building a node that would take in a user-specified goal position via a ROS service, request the planned path from the Planner node, and then publish the robot and path markers in Rviz.
- *Testing the system* - In this last part, a launch file was used to start up two robots at specific starting positions. A ROS service hosted by the Agent node was then used (via the Linux Terminal) to update the goal positions to certain values and watch as the paths were displayed on Rviz.

**System Info:** This package was tested on Ubuntu Linux 16.04 with ROS Kinetic.

#### Runtime Instructions
After cloning the repo and building it, just type `roslaunch multi_agent_planner agents.launch` into the terminal to get started. This will launch Rviz and show a 10x10 grid of purple nodes with two colored cylinders (the robots). The colors are randomly generated so if they are too similar in appearance, just `Cntrl-C` and launch the nodes again. By default, *agent_1* (the first robot) is located at position [2,0,0] and *agent_2* (the second robot) is at [0,3,0] (the order is x[m], y[m], yaw[deg]). At startup, you should see three helpful messages printed out to the console confirming that services are running. They are...
```
Motion planner service (/get_plan) is ready.
Ready to update goal pose for agent_1
Ready to update goal pose for agent_2
```
To update the pose for a robot, open up another command terminal and type...
```
rosservice call /agent_x/update_goal
```
and hit tab a couple times ('x' is a placeholder for '1', '2', '3', etc... based on which agent you would like to set the goal pose for). A 'zeroed out' `geometry_msgs::Pose2D` ROS message should appear. Fill it in with whatever pose you so choose as long as the 'x' and 'y' values are integers, and press Enter.
