# Implementation of Human-Robot Collaboration Models in Manifacturing Scenarios

Master's Thesis (M.S) for Robotics Engineering ([EMARO](https://master-emaro.ec-nantes.fr/)) program at the [University of Genova, Italy](https://unige.it/) and [Schaeffler](https://www.schaeffler.com/content.schaeffler.com/en/index.jsp).

**Supervisors**: [Prof. Fulvio Mastrogiovanni](https://www.dibris.unige.it/mastrogiovanni-fulvio) and [Dr. Kourosh Darvish](https://www.iit.it/people/kourosh-darvish)

<p align="center">
  <img src="https://user-images.githubusercontent.com/22452731/62788941-38c26400-bac8-11e9-8e76-ef5eb51a4cb7.gif" />
</p>

## Software Architecture
### Theory
The overall cooperation task is based on a modular, reactive architecture. The  architecture  has  two  phases,  namely  the Offline  Phase and  the Online  Phase.  The Offline  Phase is  composed  of teaching  safe  way-points  for  the  robot  in  the  manufacturing work-cell.   This   process   is   application-dependent   and   is done  by  a  robot  programmer.

The Online Phase has three layers namely the representation layer shown in blue,the perception layer in orange and interface layer in green. 
- Representation layer: Responsible for task representation, task allocation and task planning. 
- Perception layer: Responsible to find the object pose and grasp location to the robot.
- Interface layer: Responsible for providing interfaces to robot (robot drivers) and humans (GUIs).

<p align="center">
  <img src="https://user-images.githubusercontent.com/22452731/62839986-78897700-bc93-11e9-8ddd-9ad00ae4ad15.png" width="600" height="700" />
</p>

### External Dependency

#### Software Dependency

| Sl. No | Dependency |   Remark  |     Version    |
|:------:|:----------:|:---------:|:--------------:|
|   1.   |   [AND/OR](https://github.com/kouroshD/ANDOR)   | Mandatory |                |
|   2.   | [AI Planner](https://github.com/kouroshD/AI_Planner) | Mandatory |                |
|   3.   |     [ROS](https://www.ros.org/)    | Mandatory | Atleast Indigo |
|   4.   |     [UR Modern Driver](https://github.com/ros-industrial/ur_modern_driver)    | Mandatory | Included in this repo |
|   5.   |     [Gazebo](http://gazebosim.org/)    | Mandatory |  |

#### Hardware Dependency
The architecture is tested on UR10 Universal Robot running firmware version CB 3.0 and PolyScope version 3.5. The gripper used is OnRobot RG6 gripper and Cognex 7802 machine vision system for the perception.

### Packages
```
Configuration_files
     |
     |
     +----> and_or/files-->pallet_assembly.txt
     |
     |
     +----> sequential_planner/files----> Action_Definition_list.txt
                          |
                          |
                          +-------------> State_Action_list.txt
```
These files have to be replaced in the `AND/OR` and `AI_planner` packages. The `pallet_assembly.txt` provides the task representation, while the files `Action_Definition_list.txt` and `State_Action_list.txt` provides the necessary actions for the task manager.


#### System Setup
Linux:
```
 git clone https://github.com/prajval10/industrialRobot_task_planning.git
 cd industrialRobot_task_planning
 catkin_make
```

Add the following to your `.bashrc` file:

```
# Path to the `industrialRobot_task_planning/src` folder
export FLEX_INSTALL_PREFIX=<prefix>
# Gazebo related env variables (see http://gazebosim.org/tutorials?tut=components#EnvironmentVariables )
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${FLEX_INSTALL_PREFIX}/my_ur_gazebo/models
```






