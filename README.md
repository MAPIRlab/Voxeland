# <p align="center"> Voxeland: Probabilistic Instance-Aware Semantic Mapping with Evidence-based Uncertainty Quantification </p>

<p align="center"> <a href="https://mapir.isa.uma.es/mapirwebsite/?p=1792">Jose-Luis Matez-Bandera</a><sup>1</sup>, <a href="https://mapir.isa.uma.es/mapirwebsite/?p=1638">Pepe Ojeda</a><sup>1</sup>, <a href="https://mapir.isa.uma.es/mapirwebsite/?p=1438">Javier Monroy</a><sup>1</sup>, <a href="http://mapir.isa.uma.es/jgonzalez">Javier Gonzalez-Jimenez</a><sup>1</sup> and <a href="https://mapir.isa.uma.es/mapirwebsite/?p=1366">Jose-Raul Ruiz-Sarmiento</a><sup>1</sup></p>

<p align="center"> <sup>1</sup> Machine Perception and Intelligent Robotics (MAPIR) Group,<br/> Malaga Institute for Mechatronics Engineering and Cyber-Physical Systems (IMECH.UMA).<br/> University of Malaga. Spain. </p>

### Content
<p align="center"> <a href="#citation">Citation</a>&nbsp;&nbsp;&nbsp;<a href="#voxeland">Voxeland</a>&nbsp;&nbsp;&nbsp;<a href="#features">Features</a>&nbsp;&nbsp;&nbsp;<a href="#installation">Installation</a>&nbsp;&nbsp;&nbsp; <a href="#parameters">Parameters</a>&nbsp;&nbsp;&nbsp; <a href="#usage">Usage</a></p></p>

### Citation
<pre><code>@ARTICLE{matez_voxeland,
  author={Matez-Bandera, Jose-Luis and Ojeda, Pepe and Monroy, Javier and Gonzalez-Jimenez, Javier and Ruiz-Sarmiento, Jose-Raul},
  journal={Robotics and Autonomous Systems}, 
  title={Voxeland: Probabilistic Instance-Aware Semantic Mapping with Evidence-based Uncertainty Quantification}, 
  year={2024},
  note={Submitted and under review}}
</code></pre>


### Voxeland
Voxeland is a library for semantic mapping in voxelized worlds. It supports both dense semantics and instance-aware mapping, and is able to provide uncertainty estimations for the category labels and instance segmentation per voxel. It is designed to be used through a ROS2 node, and get new observations on-line through topics.

It is based on the excellent [Bonxai](https://github.com/facontidavide/Bonxai) library, and relies on much of its code for storing and manipulating volumetric data.

### Features
TODO

### Installation
Move to your colcon workspace and run:

```git clone --recurse-submodules git@github.com:MAPIRlab/Voxeland.git src/voxeland```

This will download the code for voxeland and some third party libraries. You will also need the `segmentation_msgs` package, available [here](https://github.com/MAPIRlab/instance_segmentation).

Once you have everything downloaded, compile with `colcon build --symlink-install` as usual.

### Parameters

## Voxeland (Launch file)
```bash

<param name="resolution" value="0.05" />                  # voxel size (in meters)
<param name="frame_id" value="map" />                     # frame_ID of the map (typically, it is "map")          
<param name="sensor_model.max_range" value="4.0" />       # maximum range of the ray to integrate information
<param name="use_height_map" value="true" />              # Publish Height colormap on Resulting VoxelGrid
<param name="semantics_as_instances" value="true" />      # Set true if considering instances in the semantic map (as explained in Voxeland's paper).
```

## Voxeland Robot Perception  (Launch file)

```bash

<!-- Dataset and Object Detector info -->
  <param name="dataset" value="other" />                          # Dataset name, to be used in modules/data_standarization.py to standarize input data.
  <param name="object_detector" value="Detectron2" />             # Object detector name, to be used in modules/data_standarization.py to standarize semantics.

<!-- Topics and Services -->
  <param name="topic_camera_info" value="/camera/camera_info" />  # Name of the topic where camera intrinsics is published (camera_info). Only necessary if intrisincs_from_topic is true.
  <param name="topic_rgb_image" value="/camera/rgb" />            # Name of the RGB images topic.
  <param name="topic_depth_image" value="/camera/depth" />        # Name of the depth images topic.
  <param name="topic_localization" value="/pose" />               # Name of the PoseWithCovarianceStamped messages topic.

<param name="semantic_segmentation_mode" value="service" />		    # Possible values: "topic" or "service", depending how the object detection method works
  <param name="service_sem_seg" value="/detectron/segment" />	    # Service name for semantic segmentation. Used if semantic_segmentation_mode="service"
  <param name="topic_sem_seg" value="/ViMantic/Detections" />	    # Topic name for semantic segmentation. Used if semantic_segmentation_mode="topic"

<!-- Images Messages Types -->
  <param name="rgb_image_type" value="Image" />                   # Possible values: "CompressedImage" or "Image", depending on the message type of the topic_rgb_image
  <param name="depth_image_type" value="Image" />                 # Possible values: "CompressedImage" or "Image", depending on the message type of the topic_depth_image

<!-- RGB-D Camera Info -->
  <param name="intrinsics_from_topic" value="true" />             # True if intrinsics published in the camera_info topic, otherwise, set to False and set intrinsics manually.
    <param name="width" value="1920" />
    <param name="height" value="1080" />
    <param name="cx" value="959.5" />
    <param name="cy" value="539.5" />
    <param name="fx" value="1371.022" />
    <param name="fy" value="1371.022" />
  <param name="camera_max_depth" value="10.0" />

<param name="limit_reliable_depth" value="true" />               # Set to true to limit the maximum reliable data from depth sensor.
<param name="min_reliable_depth" value="0.01" />                 # Minimum depth to accept points.
<param name="max_reliable_depth" value="3.00" />                 # Maximum depth to accept points.

<!-- Frame IDs -->
  <param name="map_frame_id" value="map" />                      # Map frame id
  <param name="robot_frame_id" value="camera" />                 # Robot frame id (if exists, otherwise, set equal to camera frame id)
  <param name="camera_frame_id" value="camera" />                # Camera frame id

<!-- Output configuration -->
  <param name="pointcloud_type" value="XYZRGBSemantics" />			 # Possible values: "XYZ", "XYZRGB", "XYZSemantics" and "XYZRGBSemantics".
  <param name="topic_pointcloud_output" value="cloud_in" />      # Topic where the processed PointCloud2 is published, and from which Voxeland will use as input for the mapping.  
```


### Usage

First, run the instance segmentation network (e.g., Mask R-CNN). A ROS2 implementation can be found at: [Detectron2](https://github.com/MAPIRlab/Detectron2_ros). To run detectron2 framework:

`ros2 run detectron2 detectron2_ros_node`

Next, run the robot perception node as follows:

`ros2 launch voxeland_robot_perception generic_semantic_mapping.launch.xml`

Finally, execute Voxeland to start the mapping session:

`ros2 launch voxeland bonxai_mapping.launch.xml`

(Now, everything is ready for the semantic mapping session, as soon as you play your dataset.)
