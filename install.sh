#!/usr/bin/env bash

# Install ROS Melodic

echo "Start ROS installation..."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt -y install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> $HOME/.bashrc
sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

echo "ROS has installed"

# Create workspace

echo "Installing Rust..."

curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
rustup default nightly

echo "Creating ROS workspace..."

source /opt/ros/noetic/setup.bash
mkdir -p $HOME/catkin_ws/src
cd $HOME/catkin_ws/src
catkin_init_workspace

echo "Downloading packages..."

cd ~
sudo rosdep init
sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
rosdep update
sudo apt update
sudo apt install -y python3-pip bridge-utils git
pip3 install cython empy

cd $HOME/catkin_ws/src
git clone https://github.com/clearpathrobotics/spot_ros.git
git clone https://github.com/ros/geometry2 --branch 0.6.5
git clone https://github.com/merklebot/spot_user_control.git
python3 -m pip install --upgrade pip
pip3 install -r $HOME/catkin_ws/src/spot_user_control/lease_control/requirements.txt

cd ..
rosdep install --from-paths src --ignore-src -y
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

echo "Creating service..."

cat << EOF > $HOME/run.sh
#!/usr/bin/env bash

python3 $HOME/catkin_ws/src/spot_user_control/lease_control/scripts/wait4spot.py
sleep 2
source /opt/ros/melodic/setup.bash
cd $HOME/catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source $HOME/catkin_ws/devel/setup.bash
roslaunch lease_control control.launch username:=$1 password:=$2
EOF

chmod +x $HOME/run.sh

echo "[Unit]
Description=Record rosbags and create/delete student users

[Service]
ExecStart=$HOME/run.sh
User=spot
Restart=on-failure
RestartSec=60s

[Install]
WantedBy=multi-user.target
" | sudo tee /etc/systemd/system/user_control.service

systemctl enable user_control.service
systemctl start user_control.service