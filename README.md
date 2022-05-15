# Maze solving robot

The repository contains a package named tt_213268_miniprj. In the package there is an action message called 'Start.action'. The launch file is named as 'tt_213268_miniprj.launch'.

#### General description
 
* This repository aims at controlling the turtlebot in the Gazebo           simulation to solve a maze.
* An action server called 'mini_project_action_server' and an action message called 'Start' are created after launching the package.
* The package also launches a node called 'bot_controller'.
* The node is subscribed to 'scan' topic and publishes to 'cmd_vel' topic.
* The action server will start the robot when it recieves the goal, the node reads the distance data from the 'scan' topic and it controls the movement of the robot by publishing to 'cmd_vel' topic.

#### Getting started

* Start the gazebo simulation before running the package.
* After running the project, goal should be sent to '/mini_project_action_server/goal' topic. The value of 'start' should be set 'True' in the action message.
 
 #### Problems and solutions

 * If the robot is showing some violent movements restart Gazebo simulation and try again

 #### Authors

 Tejas Takalkar



