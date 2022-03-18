# Design Project ME F376
This repository contains the necessary setup, launch files and controller codes for the multi-agent-simulation using turtlebots in ROS for the design project ME F376.

![cover_image](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/dop_ss1.png)

## 1) Running the launch files
* In the terminal run the command ``` roslaunch multi_agent_sim main.launch ``` to launch the gazebo workspace containing the turtlebots.
* Open another terminal and run the command ``` roslaunch multi_agent_sim nodes.launch ``` to launch the controller scripts for the navigation of the turtlebots.

## 2) Project Description
This project aims to simulate the behaviour of multi-robot system and understand the collective motion of the agents for path planning and obstacle avoidance. In this project, a system of 4 turtlebots are there in the gazebo workspace which is defined in the ``` multi_agent_sim/launch/main.launch ``` and accordingly the topics of the robots are defined as ```robot1/```, ```robot2/```, ```robot3/``` and  ```robot4/```.

###   File structure
![rqt_graph](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/rqt_gragh1.png)

* All the controller scripts are stored in ```multi_agent_sim/src```.
* ```controller_robot1.py``` is used to control ```robot1/``` and has a node name of _speed_controller1_.
* ```controller_robot2.py``` is used to control ```robot2/``` and has a node name of _speed_controller2_.
* ```controller_robot3.py``` is used to control ```robot3/``` and has a node name of _speed_controller3_.
* ```controller_robot4.py``` is used to control ```robot4/``` and has a node name of _speed_controller4_.
* ```controller_robot1.py``` subscribes the topic ```robot1/odom``` and publishes the topic ```robot1/cmd_vel```.
* ```controller_robot1.py``` subscribes the topic ```robot2/odom``` and publishes the topic ```robot2/cmd_vel```.
* ```controller_robot1.py``` subscribes the topic ```robot3/odom``` and publishes the topic ```robot3/cmd_vel```.
* ```controller_robot1.py``` subscribes the topic ```robot4/odom``` and publishes the topic ```robot4/cmd_vel```.
* A launchfile ```multi_agent_sim/launch/nodes.launch``` is created to launch all these nodes so that all the robots move together.

### Code functionality

