# Kimera-Semantics

<div align="center">
    <img src="docs/media/kimera_semantics.gif">
</div>

# 1. Installation

## A. Prerequisities

### i. ROS

Install ROS by following [our reference](./docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

### ii. Kimera-Semantics' dependencies

- All dependencies are listed in the `.rosinstall` files inside the `install` folder:

## B. Kimera-Semantics Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone git@github.edu:MIT-SPARK/Kimera-Semantics.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge Kimera-Semantics/install/kimera_semantics.rosinstall
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

  2. In another terminal, launch SparkVIO ROS wrapper:
  ```bash
  roslaunch spark_vio_ros spark_vio_ros_euroc.launch
  ```

  3. In another terminal, launch rviz for visualization:
  ```bash
  rviz -d $(rospack find spark_vio_ros)/rviz/spark_vio_euroc.rviz
  ```
  > Note: this rviz configuration makes use of a rviz plugin: [mesh_rviz_plugins](https://github.com/ToniRV/mesh_rviz_plugins). To visualize the textured 3D mesh, clone this plugin to your catkin workspace and catkin build it (note that this should be done automatically via `wstool`).

  4. Finally, in another terminal, launch the downloaded Euroc rosbag:
  ```bash
  rosbag play --clock /PATH/TO/EUROC_ROSBAG 
  ```

  > Note that you will need to both source ROS and your `catkin_ws` for each new terminal unless you added the following lines to your `~/.bashrc` file:
  > ```bash
  > source /opt/ros/melodic/setup.bash  # Change `melodic` for your ROS distribution.
  > source ~/catkin_ws/devel/setup.bash # Change `bash` to the shell you use.
  > ```

## Offline
  In this mode, the provided rosbag will be first parsed and then sent to the VIO for processing.
  This is particularly useful when debugging to avoid potential ROS networking issues.
  - To run, launch the SparkVIO ROS wrapper with the `online` parameter set to `false` and specify the rosbag's path:
  ```bash
  roslaunch spark_vio_ros spark_vio_ros_euroc.launch online:=false rosbag_path:="PATH/TO/ROSBAG"
  ```
