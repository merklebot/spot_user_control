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

Run `create_user` script in `lease_control` package to create new user with your ssh public key:

```console
source /opt/ros/melodic/setup.bash && source /home/spot/catkin_ws/devel/setup.bash
rosrun lease_control create_user "{'key': '<ssh_pub_key>', 'lesson': '<lesson_number>', 'e-mail': '<student_mail>'}"
[INFO] [1629206948.738579, 0.000000]: user_control ready
Enter new UNIX password: Retype new UNIX password: passwd: password updated successfully
[INFO] [1629206951.742306, 0.000000]: Created core user student_AQZ
[INFO] [1629206955.379461, 0.000000]: Created spot user student_AQZ
```

User and password to connect to Spot using SDK are in `credentials` file in the new user home directory.

To delete user run `delete_user`:

```console
source /opt/ros/melodic/setup.bash && source /home/spot/catkin_ws/devel/setup.bash
rosrun lease_control delete user
[INFO] [1629207104.224429, 0.000000]: user_control ready
[INFO] [1629207104.227672, 0.000000]: Deleted core user student_AQZ
[INFO] [1629207109.343382, 0.000000]: Deleted spot user student_AQZ
[INFO] [1629207117.359424, 0.000000]: /home/spot/student_AQZ/file1 compressed
[INFO] [1629207117.366699, 0.000000]: /home/spot/student_AQZ/file2 compressed
[INFO] [1629207118.198969, 0.000000]: Published to IPFS with hash: Qmd4TGSAxXpG5Q2U5sryTyRtjeSEYrHbXncfcQthtVnkQb
```
