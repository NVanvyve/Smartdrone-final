# Installation ROS

##### Installation on Laptop with Ubuntu
Tutorial :
https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#convenience-bash-scripts

```
$ wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh
$ sudo usermod -a -G dialout $USER
$ sudo reboot
$ source ubuntu_sim_ros_gazebo.sh
$ cd scr/Firmware
$ make posix_sitl_default gazebo
```

##### Run ROS & Gazebo on laptop

```
$ source ~/catkin_ws/devel/setup.bash
$ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
$ roslaunch px4 mavros_posix_sitl.launch
```


##### Installation on Odroid
Following this tutorial (Melodic):
http://wiki.ros.org/Installation/UbuntuARM

Date and time must be correct

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt update
$ sudo apt install ros-melodic-ros-base
$ apt search ros-melodic
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
$ mkdir -p ~/catkin_ws/src
$ cd catkin_ws/
$ catkin_make
$ sudo apt-get install ros-melodic-angles
$ sudo apt-get install ros-melodic-control-toolbox
$ sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
$ cd ~/catkin_ws
$ sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y
$ wstool init ~/catkin_ws/src
$ rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
$ rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src -j4
$ rosdep install --from-paths src --ignore-src --rosdistro melodic -y
$ cd ~/catkin_ws/src/mavros/mavros/scripts
$ sudo ./install_geographiclib_datasets.sh
$ sudo apt-get install python-serial
$ catkin build -j2 -l2
```

##### How remove ROS?
```
$ sudo apt-get remove ros-*
$ sudo apt-get purge ros-*
$ sudo rm -r /etc/ros
$ sudo apt-get autoremove
$ nano ~/.bashrc
Delete this two lines:
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rm -rf catkin_ws/
rm -rf src/Firmware
rm -rf src
```
