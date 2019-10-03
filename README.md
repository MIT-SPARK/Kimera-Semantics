# Kimera-Semantics

<div align="center">
    <img src="kimera/docs/media/kimera_semantics.gif">
</div>

# 1. Installation

## A. Prerequisities

Install ROS by following [our reference](./kimera/docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

## B. Kimera-Semantics Installation

Using [catkin](http://wiki.ros.org/catkin):

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/melodic # Change to your distro
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone git@github.edu:MIT-SPARK/Kimera-Semantics.git

# Install dependencies from rosinstall file using wstool
wstool init
# For ssh:
wstool merge Kimera-Semantics/kimera/install/kimera_semantics_ssh.rosinstall
# For https:
#wstool merge Kimera-Semantics/kimera/install/kimera_semantics_https.rosinstall
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/.bashrc
```

# 2. Usage

## Online
  1. As a general good practice, open a new terminal and run: `roscore`

  2. In another terminal, launch Kimera-Semantics:
  ```bash
  roslaunch kimera_semantics_ros kimera_semantics.launch
  ```

  3. In another terminal, launch rviz for visualization:
  ```bash
  rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics.rviz
  ```

  4. Finally, in another terminal, launch the [provided rosbag (click here to download)](https://drive.google.com/file/d/1jpuE6tMDoJyNq2Wu2EsVAc1r3e7qteUf/view?usp=sharing):
  ```bash
  rosbag play --clock $(rospack find kimera_semantics_ros)/rosbags/kimera_semantics_demo.bag
  ```

  > Note that you will need to both source ROS and your `catkin_ws` for each new terminal unless you added the following lines to your `~/.bashrc` file:
  > ```bash
  > source /opt/ros/melodic/setup.bash  # Change `melodic` for your ROS distribution.
  > source ~/catkin_ws/devel/setup.bash # Change `bash` to the shell you use.
  > ```

  3. FAQ

  - Minkindr doesn't compile:
    Catkin ignore the `minkindr_python` catkin package:
    ```
    touch ~/catkin_ws/src/minkindr/minkindr_python/CATKIN_IGNORE
    ```
