# Intelligent Robotic Manipulation - Project Lab

This repository contains the code for the practical course Intelligent Robotic Manipulation from winter semester 2023/24.

Supervisor: Snehal Jauhri

Group:
- Kaiwen Jin
- Qiuyu Xu
- Caleb Morton
- Emanuel Araujo
- Christian Maurer

# Planning

Subtasks:
- Perception (Xu, Kaiwen, Christian)
- Motion Planning (Emanuel, Caleb)
- Robot Behavior


# Package Usage (Motion Planning)

This package contains modules that facilitate handling objects with the Franka/Panda arm using Moveit. \
The modules are:
-  MoveGroupControl(eef_control.py): provides interface that allows relative low level control of the arm.
- PickAndPlace (pick_and_place.py): provides interface which allows relative high level control of the arm related to pick and place tasks, such as sending the arm to a desite position, picking an object and droping it in the desire position.
- Gripper (grasping.py): provides interface related to the Panda arm gripper, such as opening it and grasping objects.
- PlanScene (plan_scene.py): provides interface related to the Moveit planning scene, which allows checking for possible colision before making a movement.

Examples usage:
- Setting up the simulation from the Panda arm with Moveit (requires [franka_zed_gazebo](https://github.com/iROSA-lab/franka_zed_gazebo.git) and its dependencies)
```
 roslaunch franka_zed_gazebo gazebo_panda_moveit.launch 
```
- Spawn Cubes:
```
rosrun franka_zed_gazebo spawn_cubes.py
```
- Initiate node to broadcast cubes odometry into parameters:
```
rosrun irobman_project cube_param.py
```
- Initiate pick and place
```
rosrun irobman_project pick_and_place_test.py
```
