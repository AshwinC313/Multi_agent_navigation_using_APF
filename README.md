# Design Project ME F376
This repository contains the necessary setup, launch files and controller codes for the multi-agent-simulation using turtlebots in ROS for the design project ME F376.

![cover_image](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/dop_ss1.png)

## 1) Running the launch files
There are 4 different simulations in this repository. formation control of n agents (where n = 2,4 and 6). The following steps will help in running all the launch files so that each of the simulations could be seen.

### 1.1) For launching 2 agents in the workspace
* In the terminal run the command ```roslaunch multi_agent_sim main_2.launch``` to launch the gazebo workspace containing the 2 agents.
* Open another terminal and the run the command ```rosrun multi_agent_sim formation_control_2agents.py``` to run the python code which contains the control algorithm of the formation control of 2 agents.

### 1.2) For launching 4 agents in the workspace
* In the terminal run the command ```roslaunch multi_agent_sim main.launch``` to launch the gazebo workspace containing the 4 agents.
* Open another terminal and the run the command ```rosrun multi_agent_sim formation_control_4agents.py``` to run the python code which contains the control algorithm of the formation control of 4 agents.

### 1.3) For launching 6 agents in the workspace
* In the terminal run the command ```roslaunch multi_agent_sim 6_agent_main.launch``` to launch the gazebo workspace containing the 6 agents.
* Open another terminal and the run the command ```rosrun multi_agent_sim formation_control_6agents.py``` to run the python code which contains the control algorithm of the formation control of 6 agents.

## 2) Project Description
This project aims to simulate the behaviour of multi-robot system and understand the collective motion of the agents for path planning and obstacle avoidance. In this project, a system of n turtlebots (n = 2, 4, 6) are there in the gazebo workspace.
* For n = 2, we have to use a launch file of ```multi_agent_sim/main_2.launch``` which have robots defined as ```/robot1``` and ```/robot2```.
* For n = 4, we have to use a launch file of ```multi_agent_sim/main.launch``` which have robots defined as ```/robot1``` , ```/robot2```, ```/robot3``` and ```/robot4```.
* For n = 4, we have to use a launch file of ```multi_agent_sim/6_agent_main.launch``` which have robots defined as ```/robot1``` , ```/robot2```, ```/robot3``` ,```/robot4```, ```/robot5``` and ```/robot6```.


###   File structure

#### RQT Graph of 2 agents
![rqt_graph_2_agents](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/rqt_graph_for_2agents.png)

#### RQT Graph of 4 agents
![rqt_graph_4_agents](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/rqt_graph_for_4agents.png)

#### RQT Graph of 6 agents
![rqt_graph_6_agents](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/rqt_graph_for_6agents.png)

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
The script is provided with the goal coordinates and the agent moves towards the goal by calculating the error in the orientation and the distance between the goal and initial position.

####  Fig. 1 Initial orientation of the system
![initial_orientation](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/initial_orientation.png)

#### Fig. 2 Video of how the robots are moving
![video_simulation](https://github.com/AshwinC313/Design_Project_MEF376/blob/main/simulation.gif.gif)





