# Voxeland
Voxeland is a library for semantic mapping in voxelized worlds. It supports both dense semantics and instance-aware mapping, and is able to provide uncertainty estimations for the category labels and instance segmentation per voxel. It is designed to be used through a ROS2 node, and get new observations on-line through topics.

It is based on the excellent [Bonxai](https://github.com/facontidavide/Bonxai) library, and relies on much of its code for storing and manipulating volumetric data.

## Features
TODO

## Installation
Move to your colcon workspace and run:

```git clone --recursive git@github.com:MAPIRlab/Voxeland.git src/voxeland```

This will download the code for voxeland and some third party libraries. You will also need the `segmentation_msgs` package, available [here](https://github.com/MAPIRlab/instance_segmentation).

Once you have everything downloaded, compile with `colcon build --symlink-install` as usual.

## Usage
TODO