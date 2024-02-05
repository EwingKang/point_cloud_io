Point Cloud IO
======================

Overview
---------------

These are two simple [ROS2] point cloud helper nodes. **_read_** reads a point cloud from file (ply or vtk) and publishes it as a [sensor_msgs/msg/PointCloud2] message. **_write_** subscribes to a [sensor_msgs/msg/PointCloud2] topic and writes received messages to seperate files (ply, pcd).

For visualization, make sure to set the **Decay Time** in the **PointCloud2** tab in [rviz2] to a high number to get the point cloud visible for a long time.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Péter Fankhauser, Remo Diethelm<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Remo Diethelm, rdiethelm@anybotics.com<br />
ROS2 adaptation: Ewing Kang f039281310@yahoo.com.tw<br />**

This projected was initially developed at ETH Zurich (Autonomous Systems Lab & Robotic Systems Lab).

[This work is conducted as part of ANYmal Research, a community to advance legged robotics.](https://www.anymal-research.org/)

The source code is released under a [BSD 3-Clause license](LICENSE).

Installation
------------

### Dependencies

This software is built on the Robot Operating System ([ROS2]), which needs to be [installed](https://docs.ros.org/en/) first. Additionaly, the it depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/).


### Building

In order to build Point Cloud IO, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd ~/ros2/src
    git clone -b texture_mesh git@glorymakeup.com:robotic_software_integration/pointcloud_io.git
    cd ../
    colcon build  --packages-select pointcloud_io

Note: building the tool with support for the VTK file format is disabled by default. To enable it, run `colcon build --packages-select pointcloud_io --cmake-args -DBUILD_WITH_VTK_SUPPORT=True` instead.

Usage
------------

To create your own launch-file, you can use the examples from `pointcloud_io/launch/...`.


### Read

Load and publish a ply or vtk file with
```bash
# mimimal launch
ros2 run pointcloud_io read --ros-args -p file_path:=/home/user/my_point_cloud.ply -p topic:=/my_topic -p frame:=/sensor_frame

# Launch with scale, rate, rotation, and mesh
ros2 run pointcloud_io read --ros-args -p file_path:=src/spray_simulator/assets/Peggie_50K.obj \
-p topic:=/mkd/user/pointcloud \
-p mesh_topic:=/mkd/user/mesh \
-p texturemesh_topic:=/mkd/user/textured_mesh \
-p frame_id:=/physiog \
-p rate:=2.0 -p scale:=0.001 -p rpy_deg:=[0.0,0.0,90.0]

```

Optionally, you can also add `-p rate:=1.0` to have the node publish your point cloud at the specified rate.
Optionally, you can also add `-p scale:=0.01` to uniformly scale your model.


### Write

Subscribe and save point clouds to a ply file with

    rosrun pointcloud_io write  --ros-args -p topic:=/your_topic -p folder_path:=/home/user/my_point_clouds

Optionally, you can set parameters to fit the point cloud file names to your needs:

- `-p file_prefix:=my_prefix` (default: "point_cloud")
- `-p file_ending:=my_ending` (default: "ply", currently only format which is supported for writing)
- `-p add_counter_to_path:=false` (default: `true`)
- `-p add_frame_id_to_path:=true` (default: `false`)
- `-p add_stamp_sec_to_path:=true` (default: `false`)
- `-p add_stamp_nsec_to_path:=true` (default: `false`)


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://git.glorymakeup.com/robotic_software_integration/pointcloud_io/issues).


[ROS]: http://www.ros.org
[rviz]: https://index.ros.org/p/rviz2
[sensor_msgs/PointCloud2]: https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html
