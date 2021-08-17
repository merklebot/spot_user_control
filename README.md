# Spot User Control

Packages for manipulating with users and recording rosbag backup files for Spot.

## Requirements

* [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [Spot SDK](https://github.com/boston-dynamics/spot-sdk/blob/master/docs/python/quickstart.md)
* Clearpath [Spot ROS package](https://clearpathrobotics.com/assets/guides/melodic/spot-ros/ros_setup.html)

## Installation 

Clone repository to your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/merklebot/spot_user_control.git
```
Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

Run `control.launch` file for recording logs and run e-stop (it is running on spot):
```bash
roslaunch lease_control control.launch
```
Run `create_user.py` script in `lease_control` package to create new user with your ssh public key:
```bash
roscd lease_control
python3 scripts/create_user.py <ssh public key>
```
User and password to connect to Spot using SDk are in `credentials` file in the new user home directory.
To delete user run `delete_user.py`:
```bash
roscd lease_control
python3 scripts/delete_user.py <username>
```
