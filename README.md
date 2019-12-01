# Kimera-Semantics

<div align="center">
    <img src="kimera/docs/media/kimera_semantics.gif">
</div>

## Release News

- **Dec 1st 2019:** Kimera-Semantics got a complete revamp:
  - Leaner code: no more code dedicated to meshing, we fully re-use Voxblox/OpenChisel instead.
  - New `fast` method: an order of magnitude faster (took approx 1s before, 0.1s now) than using `merged`, with minimal accuracy loss for small voxels (it leverages Voxblox' fast approach):
  you can play with both methods by changing the parameter `semantic_tsdf_integrator_type` in the [launch file](./kimera_semantics_ros/launch/kimera_semantics.launch).
<div align="center">
    <img src="kimera/docs/media/fast_vs_merged_kimera_semantics.gif">
</div>

## Publications

We kindly ask to cite our paper if you find this library useful:

 - A. Rosinol, M. Abate, Y. Chang, L. Carlone. [**Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping**](https://arxiv.org/abs/1910.02490). arXiv preprint [arXiv:1910.02490](https://arxiv.org/abs/1910.02490).
 ```bibtex
 @misc{Rosinol19arxiv-Kimera,
   title = {Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping},
   author = {Rosinol, Antoni and Abate, Marcus and Chang, Yun and Carlone, Luca},
   year = {2019},
   eprint = {1910.02490},
   archiveprefix = {arXiv},
   primaryclass = {cs.RO},
   url = {https://github.com/MIT-SPARK/Kimera},
   pdf = {https://arxiv.org/pdf/1910.02490.pdf}
 }
```

### Related publications

Our work is built using [Voxblox](https://github.com/ethz-asl/voxblox), an amazing framework to build your own 3D voxelized world:

- Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, [**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**](https://github.com/ethz-asl/voxblox), in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2016.

Which was originally inspired by [OpenChisel](https://github.com/personalrobotics/OpenChisel):

- Matthew Klingensmith, Ivan Dryanovski, Siddhartha Srinivasa, and Jizhong Xiao, [**Chisel: Real Time Large Scale 3D Reconstruction Onboard a Mobile Device using Spatially Hashed Signed Distance Fields.**](http://www.roboticsproceedings.org/rss11/p40.pdf). Robotics: science and systems (RSS), 2015.

A related work to ours is [Voxblox++](https://github.com/ethz-asl/voxblox-plusplus) which also uses Voxblox for geometric and instance-aware segmentation, differently from our dense scene segmentation, check it out as well!:

- Margarita Grinvald, Fadri Furrer, Tonci Novkovic, Jen Jen Chung, Cesar Cadena, Roland Siegwart, and Juan Nieto, [**Volumetric Instance-Aware Semantic Mapping and 3D Object Discovery**](https://github.com/ethz-asl/voxblox-plusplus), in IEEE Robotics and Automation Letters, July 2019.

# 1. Installation

## A. Prerequisities

- Install ROS by following [our reference](./kimera/docs/ros_installation.md), or the official [ROS website](https://www.ros.org/install/).

- Install system dependencies:
```bash
sudo apt-get install python-wstool python-catkin-tools  protobuf-compiler autoconf
# Change `melodic` below for your own ROS distro
sudo apt-get install ros-melodic-cmake-modules
```

## B. Kimera-Semantics Installation

Using [catkin](http://wiki.ros.org/catkin):

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/melodic # Change `melodic` to your ROS distro
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/Kimera-Semantics.git

# Install dependencies from rosinstall file using wstool
wstool init # Use unless wstool is already initialized

# For ssh:
wstool merge Kimera-Semantics/kimera/install/kimera_semantics_ssh.rosinstall
# For https:
#wstool merge Kimera-Semantics/kimera/install/kimera_semantics_https.rosinstall

# Download and update all dependencies
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build kimera

# Refresh workspace
source ~/catkin_ws/devel/setup.bash
```

# 2. Usage

  0. Download the [demo rosbag (click here to download)](https://drive.google.com/open?id=1jpuE6tMDoJyNq2Wu2EsVAc1r3e7qteUf) and save it inside the rosbag folder in the kimera folder: `./kimera/rosbag/kimera_semantics_demo.bag`.

  1. As a general good practice, open a new terminal and run: `roscore`

  2. In another terminal, launch the [provided rosbag (click here to download)](https://drive.google.com/open?id=1jpuE6tMDoJyNq2Wu2EsVAc1r3e7qteUf):
  ```bash
  rosbag play --clock --pause $(rosstack find kimera)/rosbags/kimera_semantics_demo.bag
  ```

  3. In another terminal, launch Kimera-Semantics:
  ```bash
  roslaunch kimera_semantics_ros kimera_semantics.launch
  ```

  4. In another terminal, launch rviz for visualization:
  ```bash
  rviz -d $(rospack find kimera_semantics_ros)/rviz/kimera_semantics_gt.rviz
  ```

  Now, go back to the terminal where the rosbag was launched and just press `space` to run the rosbag.

  > Note: you will need to source your `catkin_ws` for each new terminal unless you added the following line to your `~/.bashrc` file:
  > ```bash
  > source ~/catkin_ws/devel/setup.bash # Change `bash` to the shell you use.
  > ```

  > Note 2: you might need to check/uncheck once the `Kimera Semantic 3D Mesh` left pane topic in rviz.

  # 3. FAQ

  - Minkindr doesn't compile:
    Catkin ignore the `minkindr_python` catkin package:
    ```
    touch ~/catkin_ws/src/minkindr/minkindr_python/CATKIN_IGNORE
    ```
