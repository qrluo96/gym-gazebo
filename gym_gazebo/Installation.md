完全按照这些步骤来，就不会出问题，也不需要修改makefile

（虽然不知道为什么

# Check python version
```
Python -V
```
Must be python2 (don't know why

# Install ROS Melodic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	
sudo apt update
	
sudo apt install ros-melodic-desktop-full
	
sudo rosdep init
```

    rosdep update
	
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

    source ~/.bashrc
	
	sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
	
# Gym-gazebo
ROS Melodic related dependencies
```
sudo apt-get install \
python-pip python3-vcstool python3-pyqt4 \
pyqt5-dev-tools \
libbluetooth-dev libspnav-dev \
pyqt4-dev-tools libcwiid-dev \
cmake gcc g++ qt4-qmake libqt4-dev \
libusb-dev libftdi-dev \
python3-defusedxml python3-vcstool \
ros-melodic-octomap-msgs        \
ros-melodic-joy                 \
ros-melodic-geodesy             \
ros-melodic-octomap-ros         \
ros-melodic-control-toolbox     \
ros-melodic-pluginlib	       \
ros-melodic-trajectory-msgs     \
ros-melodic-control-msgs	       \
ros-melodic-std-srvs 	       \
ros-melodic-nodelet	       \
ros-melodic-urdf		       \
ros-melodic-rviz		       \
ros-melodic-kdl-conversions     \
ros-melodic-eigen-conversions   \
ros-melodic-tf2-sensor-msgs     \
ros-melodic-pcl-ros \
ros-melodic-navigation \
ros-melodic-sophus
```
	
Install Python Packages:
```
sudo pip install gym h5py tensorflow-gpu keras
sudo apt-get install python-skimage
```
	
Install gym-gazebo
```
cd ~
git clone https://github.com/erlerobot/gym-gazebo
cd gym-gazebo
sudo pip install -e .
```
	
Run bash files, build the ros workspace:
```
cd gym_gazebo/envs/installation
bash setup_melodic.bash
```
	
# Test
Run example for qlearn:

Terminal 1
```
cd gym-gazebo/gym_gazebo/envs/installation/
bash turtlebot_setup.bash
```
Terminal 2
```
cd gym-gazebo/examples/turtlebot
python circuit_turtlebot_lidar_qlearn.py
```
Terminal 3
```
cd gym-gazebo/gym_gazebo/envs/installation/
source turtlebot_setup.bash
export GAZEBO_MASTER_URI=http://localhost:xxxxx
gzclient
```
	
