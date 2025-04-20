# Orca ROV Perception Package

This package contains the perception system for the Orca ROV.

## Overview

The Orca ROV perception package provides:
- Length measurement using point cloud data
- Object detection using computer vision

## Nodes

### Length Measurement Node

The length measurement node processes point cloud data from the RealSense camera to measure object lengths.

```bash
rosrun orca_perception length_measurement_node
```

#### Subscribed Topics
- `/realsense/depth/points` (sensor_msgs/PointCloud2): Point cloud data from the RealSense camera

#### Published Topics
- `object_length` (std_msgs/Float64): Measured object length
- `filtered_cloud` (sensor_msgs/PointCloud2): Filtered point cloud
- `segmented_cloud` (sensor_msgs/PointCloud2): Segmented object point cloud

#### Parameters
- `min_distance` (double, default: 0.1): Minimum distance for filtering (meters)
- `max_distance` (double, default: 2.0): Maximum distance for filtering (meters)
- `cluster_tolerance` (double, default: 0.02): Clustering tolerance (meters)
- `min_cluster_size` (int, default: 100): Minimum cluster size (points)
- `max_cluster_size` (int, default: 25000): Maximum cluster size (points)

### Object Detection Node

The object detection node processes camera images to detect objects using computer vision.

```bash
rosrun orca_perception object_detection.py
```

#### Subscribed Topics
- Camera image topic (specified by parameter, default: `/front_camera/image_raw`)

#### Published Topics
- `detection_image` (sensor_msgs/Image): Image with detection overlays
- `detected_object` (geometry_msgs/Point): Position of detected object
- `object_type` (std_msgs/String): Type of detected object

#### Parameters
- `camera_topic` (string, default: "/front_camera/image_raw"): Camera topic to subscribe to
- `detection_method` (string, default: "contour"): Detection method ("contour" or "color")
- `min_area` (int, default: 1000): Minimum contour area (pixels)
- `target_color_lower` (list, default: [0, 100, 100]): HSV lower bound for color detection
- `target_color_upper` (list, default: [10, 255, 255]): HSV upper bound for color detection

## Launch Files

### perception.launch

Launches the complete perception system.

```bash
roslaunch orca_perception perception.launch
```

## Configuration

The perception system is configured via the `perception.yaml` file in the `config` directory. This file includes:
- Length measurement parameters
- Object detection parameters

## Usage Examples

### Visualizing Object Detection

To visualize the object detection results:
```bash
rosrun rqt_image_view rqt_image_view
```
Then select the `/detection_image` topic.

### Visualizing Point Clouds

To visualize the point clouds:
```bash
rosrun rviz rviz
```
Then add a PointCloud2 display and select the `/filtered_cloud` or `/segmented_cloud` topic.

## Algorithm Details

### Length Measurement

The length measurement algorithm works as follows:
1. Filter the point cloud to remove noise and limit the distance range
2. Segment the filtered point cloud into clusters using Euclidean clustering
3. Find the largest cluster (assumed to be the object of interest)
4. Calculate the bounding box dimensions of the cluster
5. Return the maximum dimension as the object length

### Object Detection

The object detection algorithm supports two methods:
1. Contour-based detection:
   - Convert image to grayscale
   - Apply Gaussian blur
   - Apply Canny edge detection
   - Find contours
   - Analyze contour shape to determine object type

2. Color-based detection:
   - Convert image to HSV color space
   - Create a mask for the target color range
   - Apply morphological operations to remove noise
   - Find contours in the mask
   - Identify objects based on contour properties
