# Design Project ME F376
This repository contains the necessary setup, launch files and controller codes for the multi-agent-simulation using turtlebots in ROS for the design project ME F376
![cover_image](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/dop_ss1.png)

## 1) Running the launch files
* In the terminal run the command ``` roslaunch multi_agent_sim main.launch ``` to launch the gazebo workspace containing the turtlebots.
* Open another terminal and run the command ``` roslaunch multi_agent_sim nodes.launch ``` to launch the controller scripts for the navigation of the turtlebots.

## 2) Project Description
This project aims to simulate the behaviour of multi-robot system and understand the collective motion of the agents for path planning and obstacle avoidance. In this project, a system of 4 turtlebots are there in the gazebo workspace which is defined in the ``` multi_agent_sim/launch/main.launch ``` and accordingly the topics of the robots are defined as ```robot1/```, ```robot2/```, ```robot3/``` and  ```robot4/```.

###   File structure
![rqt_graph](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/rqt_gragh1.png)
