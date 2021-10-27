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
source /opt/ros/melodic/setup.bash && source /home/spot/catkin_ws/devel/setup.bash && source /root/.env
rosrun lease_control create_user create_user --session-id 33 --ssh-keys <ssh_key1> <ssh_key2> --lesson <lesson_number> --emails <email1> <email2>
[INFO] [1635319684.741609]: user_control ready
Create user: started
Enter new UNIX password: Retype new UNIX password: passwd: password updated successfully
[INFO] [1635319687.765813]: Created core user student
[INFO] [1635319691.882607]: Created spot user student
State: started
```

User and password to connect to Spot using SDK are in `credentials` file in the new user home directory.

To delete user run `delete_user`:

```console
source /opt/ros/melodic/setup.bash && source /home/spot/catkin_ws/devel/setup.bash && source /root/.env
rosrun lease_control delete_user --session-id <session_id>
[INFO] [1635319701.881664]: user_control ready
Delete user: started
[INFO] [1635319701.889975]: Deleted core user student
[INFO] [1635319706.458831]: Deleted spot user student
State: ipfs
Send to IPFS: ipfs
[INFO] [1635319706.469890]: /home/spot/student/metadata compressed
[INFO] [1635319710.348172]: Published to IPFS with hash: QmWmD2jLa5eRhHTMkfTfJY2rrt6HFQ2XjvSoG29uV6ntLS
State: blockchain
Send datalog: blockchain
[INFO] [1635319711.008685]: DataLogger instantiated
[INFO] [1635319711.010844]: Sending to datalog: QmWmD2jLa5eRhHTMkfTfJY2rrt6HFQ2XjvSoG29uV6ntLS
[INFO] [1635319738.042535]: Datalog created with extrinsic hash: 0xba28a4d20d0812e30b1d8703a78933b5c9c071943370a7a05490c69602e3b114
DataLog Extrinsic Hash: 0xba28a4d20d0812e30b1d8703a78933b5c9c071943370a7a05490c69602e3b114
State: finished
IPFS hash: QmWmD2jLa5eRhHTMkfTfJY2rrt6HFQ2XjvSoG29uV6ntLS, extrinsic hash: 0xba28a4d20d0812e30b1d8703a78933b5c9c071943370a7a05490c69602e3b114
```
