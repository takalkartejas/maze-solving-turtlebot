# Maze solving robot

The repository contains a package named tt_213268_miniprj. In the package there is an action message called 'Start.action'. The launch file is named as 'tt_213268_miniprj.launch'.

## General description
 
* This repository aims at controlling the turtlebot in the Gazebo simulation to solve a maze. This task is achieved by the package called tt_213268_miniprj. The algorithm is to simply keep following the wall. The package called miniproject_world contains a sample world with a maze and an obstacle blocking the maze.  A sample of the simulation of robot getting out of maze is shown below.
  
https://user-images.githubusercontent.com/67382565/205753830-bee16a42-9b9a-4410-b707-785b234e486b.mp4

## Implementation steps
Following steps were taken to implement this project
1. Implement an action server such that the user must publish **start_driving: true"** as a goal to start the robot.
2. Move the robot till it detects the wall
3. Delete the obstacle using '/gazebo/delete_model' service
4. Follow the wall till it gets out of the maze
   
## Project structure
The following screenshot of rqt_graph shows the structure of the control system

![total_control_system](https://user-images.githubusercontent.com/67382565/205900587-6d5e9309-4692-462a-91d3-78a09cb988df.png)

* The package tt_213268_miniprj contains a node called 'bot_controller' which publishes the linear and angular velocities of robot to '/cmd_vel' topic and is subscribed to '/scan' topic which gives the data from lidar sensor.
* The above mentioned structure allows the node to control robot and get info about its position relative to the enviournment.
* An action server called mini_project_action_server is created by the node. As shown in the figure no. 2 the user publishes to its goal from the terminal. This is done to start moving the robot.
* The package also contains an action messege called 'Start' which is used by the action server.
  
## Dependencies 
* Ubuntu 20.04
* ROS Noetic
* Gazebo
* Turtlebot3 Packages
* Python 3.8.10
* Python Packages: numpy, math, time

## Getting started
* If you have valid ros installation along with Gazebo anf turtlebot3 packages, copy this packages inside the src  directory of your catkin workspace.
* Start the gazebo simulation with the maze world.
    ```bash
    roslaunch miniproject_world world.launch
    ```
* Start the robot_control node using the launch file in new terminal
    ```bash
  roslaunch tt_213268_miniprj start.launch
    ```
* Publish goal to action server in new terminal
    ```bash
    rostopic pub /mini_project_action_server/goal tt_213268_miniprj/StartActionGoal "header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    goal_id:
      stamp:
        secs: 0
        nsecs: 0
      id: ''
    goal:
      start_driving: true"
    ```

 



