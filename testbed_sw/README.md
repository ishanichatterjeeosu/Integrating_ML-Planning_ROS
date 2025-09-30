# Testbed Motion Planning Software

This repository contains the ROS packages necessary for motion planning for the testbed

## Docker Support

Under `docker`, we have included a DockerFile for setting up an image running ROS Noetic with MoveIt! installed.
Please see the `README.md` under `docker` to see how to use these DockerFile if interested.

## Dependencies

Usage of this repository requires the installation of ROS Noetic, Rviz, Gazebo 11, and MoveIt! Which can be installed via the following commands:

Install ROS Noetic, RViz, Gazebo, and MoveIt!
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
# If you know what you're doing, you can get way with ros-noetic-desktop and installing required packages individually, see the DockerFile for list of individual packages
sudo apt install ros-noetic-desktop-full python3-catkin-tools ros-noetic-moveit 
```
Note that the dependencies list was not tested, as the main development was done with Docker containers. We believe the above instructions give all packages necessary, but there may be other dependencies not reported. Please submit a pull request if you find missing dependencies.

## Installation

1. Clone the repository inside of your catkin workspace. (If you do not have a catkin workspace set up, follow the instructions from the [ROS wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))
2. Run `catkin build`
3. Source the setup.bash file. `source devel/setup.bash`

## Testbed Only Examples

### Show testbed in Rviz with sliders to adjust joints

```bash
roslaunch testbed_description rviz.launch
```

### Perform motion planning with fake controller in RViz only
```bash
roslaunch testbed_moveit_config demo.launch
```

### Perform motion planning with a simple controller with fake controller in RViz only
```bash
roslaunch testbed_moveit_config demo.launch moveit_controller_manager:=simple
```
Assumes there is a `FollowJointTrajectory` action server at the namespace `testbed_controller/follow_joint_trajectory`. 
This can be changed in `testbed_moveit_config/config/simple_moveit_controllers.yaml`

### Perform motion planning with roscontrol with fake controller in RViz only
```bash
roslaunch testbed_moveit_config demo.launch moveit_controller_manager:=ros_control
```
Assumes there is a `effort_controllers/JointTrajectoryController` that can control joints `joint_x`, `joint_y`, `joint_z`, `joint_yaw`. This can be modified in `testbed_moveit_config/config/ros_controllers.yaml`


### Show testbed in Gazebo

```bash
roslaunch testbed_description gazebo.launch
```

### Perform motion planning with simple controller with controller in Gazebo
```bash
roslaunch testbed_moveit_config testbed_moveit.launch
```
This script uses the testbed controller in `testbed_control` package to instantiate a `JointTrajectoryController` that is then hooked into the `ros_control` MoveIt! interface.
As a result, it uses the config file in `testbed_control` instead of in `moveit_config` package.

```bash
roslaunch testbed_moveit_config demo_gazebo.launch
```
This script uses the testbed controller defined in `testbed_moveit_config/config/ros_controllers.yaml` named `joint_trajectory_controller`. After validating this works, the `testbed_control` package may be unnecessary.

## Testbed with Arm Examples

### Show testbed in Rviz with sliders to adjust joints

```bash
roslaunch testbed_bravo_description rviz.launch
```

### Perform motion planning with fake controller in RViz only
```bash
roslaunch testbed_bravo_moveit_config demo.launch
```


# Notes

## Buggy Rviz
If the Rviz windows are appearing broken or not rendering correctly, [turning off hardware acceleration](http://wiki.ros.org/rviz/Troubleshooting) could help:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```