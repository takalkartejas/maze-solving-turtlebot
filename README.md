# Maze solving robot

The repository contains a package named tt_213268_miniprj. In the package there is an action message called 'Start.action'. The launch file is named as 'tt_213268_miniprj.launch'.

## General description
 
* This repository aims at controlling the turtlebot in the Gazebo simulation to solve a maze. This task is achieved by the package called tt_213268_miniprj. The algorithm is to simply keep following the wall. The package called miniproject_world contains a sample world with a maze and an obstacle blocking the maze.  A sample of the simulation of robot getting out of maze is shown below.
  
https://user-images.githubusercontent.com/67382565/205753830-bee16a42-9b9a-4410-b707-785b234e486b.mp4


## Project structure
The following screenshot of gitg output shows the structure of the control system

#### Getting started

* Start the gazebo simulation before running the package.
* After running the project, goal should be sent to '/mini_project_action_server/goal' topic. The value of 'start' should be set 'True' in the action message.
 
 #### Problems and solutions

 * If the robot is showing some violent movements restart Gazebo simulation and try again

 #### Authors

 Tejas Takalkar



