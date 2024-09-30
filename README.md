# Voxeland
Voxeland is a library for semantic mapping in voxelized worlds. It supports both dense semantics and instance-aware mapping, and is able to provide uncertainty estimations for the category labels and instance segmentation per voxel. It is designed to be used through a ROS2 node, and get new observations on-line through topics.

It is based on the excellent [Bonxai](https://github.com/facontidavide/Bonxai) library, and relies on much of its code for storing and manipulating volumetric data.

## Features
TODO

## Installation
Move to your colcon workspace and run:

```git clone --recurse-submodules git@github.com:MAPIRlab/Voxeland.git src/voxeland```

This will download the code for voxeland and some third party libraries. You will also need the `segmentation_msgs` package, available [here](https://github.com/MAPIRlab/instance_segmentation).

Once you have everything downloaded, compile with `colcon build --symlink-install` as usual.

## Usage

First, run the instance segmentation network (e.g., Mask R-CNN). A ROS2 implementation can be found at: [Detectron2](https://github.com/MAPIRlab/Detectron2_ros). To run detectron2 framework:

`ros2 run detectron2 detectron2_ros_node`

Next, run the robot perception node as follows:

`ros2 launch voxeland_robot_perception robot_perception_node.py`

Finally, execute Voxeland to start the mapping session:

`ros2 launch voxeland bonxai_mapping.launch.xml`

(Now, everything is ready for the semantic mapping session, as soon as you play your dataset.)
