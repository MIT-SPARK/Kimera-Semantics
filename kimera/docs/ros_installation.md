## ROS installation

Install [ROS Desktop-Full Install](http://wiki.ros.org/kinetic/Installation), below we prodive installation instructions:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
```

Now, you can install the ROS distribution corresponding to your system.

- If you have **Ubuntu 14.04**, run:
```
# Install ROS distribution depending on your system: Ubuntu 14.04 -> kinetic, 16.04 -> melodic
sudo apt-get install ros-kinetic-desktop-full
# Automatically source ROS for convenience:
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- Otherwise, if you have **Ubuntu 16.04**, run:
```
# Install ROS distribution depending on your system: Ubuntu 14.04 -> kinetic, 16.04 -> melodic
sudo apt-get install ros-melodic-desktop-full
# Automatically source ROS for convenience:
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Now, initialize rosdep:
```
sudo rosdep init
rosdep update
```

Finally, install dependencies for building packages and catkin tools:
```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
```
