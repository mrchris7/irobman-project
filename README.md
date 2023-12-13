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
- CV, Tracking, Object detection, RGBD-Cube Detector 2D/3D (Xu, Kaiwen, Christian)
- Motion Planning (Emanuel, Caleb)

- (Behavior Tree (high level task))

## To do
- Motion Planning 
    - [ ] Edit pick_and_place_test.py in order to make the arm align with the cube when grasping it. It already goes to the cube position, but we need to align the YAW from the gripper and the cube;
    - [ ] Make better grasp approach, currently the Gripper aproach the cube from up, but it would be better if it comes from the side in order to avoid pressing the cube against the table;
    - [ ] Make algorith that generate the stucture with the cubes, like a list with positions;
    - [ ] Make pipeline. 
        - First phase would be having a better view from the table and detect the cubes;
        - Second phase would be picking and placing the target cube to desire state;
        - Third phase would be checking if moving was succesful;
        - Reset loop until the desite structure is complete.

# Package Usage

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
