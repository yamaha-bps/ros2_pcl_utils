# ```ros2_pcl_utils```

[![foxy](https://github.com/yamaha-bps/ros2_pcl_utils/actions/workflows/foxy.yaml/badge.svg)](https://github.com/yamaha-bps/ros2_pcl_utils/actions/workflows/foxy.yaml) [![galactic](https://github.com/yamaha-bps/ros2_pcl_utils/actions/workflows/galactic.yaml/badge.svg)](https://github.com/yamaha-bps/ros2_pcl_utils/actions/workflows/galactic.yaml)

## Installation

Clone into a ```colcon``` workspace and build as usual.

```
git clone https://github.com/yamaha-bps/ros2_pcl_utils.git src/pcl_utils 
colcon build
```

## Usage

Nodes can be started directly as standalone executables,
```
ros2 run pcl_utils feature_node         # executable for PclFeatureNode
ros2 run pcl_utils seg_node             # executable for PclSegNode
ros2 run pcl_utils image_overlay_node   # executable for PclImageOverlayNode
```
or by loading them in a container:
```
# start container
ros2 run rclcpp_components component_container --ros-args -r __node:=my_container
# load nodes into container
ros2 component load /my_container pcl_utils cbr::PclFeatureNode
ros2 component load /my_container pcl_utils cbr::PclSegNode
ros2 component load /my_container pcl_utils cbr::PclImageOverlayNode
```

## Example

These images are from the ```launch/demo.launch.py``` example.

To run the demo:
```
source /usr/share/gazebo/setup.sh  # may be required by gazebo
ros2 launch pcl_utils demo.launch.py
```

Consider this gazebo scene with one camera and one lidar viewing two objects:

<img src="images/scene.png" width="500">

The ```PclFeatureNode``` extracts edge and planar features from the lidar pointcloud. The first image shows
the raw pointcloud, and the second shows features extracted by ```PclFeatureNode```.

<img src="images/pcl.png" width="400"> <img src="images/features.png" width="400">

The ```PclImageOverlayNode``` projects a pointcloud onto an image.

<img src="images/image.png" width="400"> <img src="images/overlay.png" width="400">

## Nodes

The nodes assume that the incoming pointclouds have the XYZI structure.

Points with negative intensity are assumed to be non-returns.

### ```cbr::PclFeatureNode```

Extracts edge and planar features from a pointcloud.

NOTE The incoming pointcloud must consist of consecutive laser points from the same laser,
or the algorithms will not work properly.

Features are extracted by

1. filtering out points that are invalid, for example due to
  - Risk being obscured (near a point that is closer to the sensor)
  - Too far or too close
  - Angle from sensor principal axis too large
  - Surface inclination angle too large
  - Close to scan discontinuity

2. Computing **c-values** for remaining points that determine how close a point
is to being the average of its surrounding. Small c-values are indicative of a 
planar surface, while high c-values are indicative of an edge.


![](images/equation.png)

#### Subscribes to

- ```pointcloud``` - ```sensor_msgs/msg/PointCloud2```

#### Publishes to

- ```feature/plane``` - ```sensor_msgs/msg/PointCloud2```
- ```feature/edge``` - ```sensor_msgs/msg/PointCloud2```

#### Parameters

- ```window``` - ```int```, default ```5```

  Window width (in points) in feature extraction algorithm.
  At most one feature is detected in each window.

- ```ang_disc_thresh``` - ```float```, default ```M_PI```

  Angular discontinuity threshold [rad] (between 0 and PI).
  Points in window around a discontinuity are marked as invalid.
  Leave as M_PI if scan is known to be continuous.

- ```rel_disc_thresh``` - ```float```, default ```MAX_FLOAT```

  Relative distance discontinuity threshold.
  Points in window on the far side of a relative discontinuity are marked as invalid
  since they may be occluded by a small sensor movement.

- ```ang_inc_thresh``` - ```float```, default ```M_PI_2```

  Max angle of ray w.r.t. Y-Z plane [rad] (between 0 and PI / 2).
  A small angle indicates that the surface is facing the lidar.
  Points with high angle angle on both sides are marked as ineligible.

- ```max_angle``` - ```float```, default ```M_PI```

  Max angle w.r.t. lidar x axis [rad] (between 0 and PI).
  Points with larger angle w.r.t. lidar center are marked as ineligible.

- ```min_depth```, ```max_depth```, ```float```, default ```0``` and ```MAX_FLOAT```

  Min and max distance from sensor for valid features.

- ```min_intensity```, ```max_intensity```, ```float```, default ```0``` and ```MAX_FLOAT```

  Intensity range for valid points.

- ```plane_thresh``` - ```float```, default ```2e-4```

  Upper cvalue threshold for planar features.

- ```edge_thresh``` - ```float```, default ```1e-2```

  Lower cvalue threshold for edge features.


### ```cbr::PclSegNode```

Filters an incoming pointcluod w.r.t. a segmentation mask. Only points that
project inside the image to a pixel that is in one of the specified segmentation classes
are re-published.

Non-returns (negative intensity) are re-published.

#### Subscribes to

- ```pointcloud``` - ```sensor_msgs/msg/PointCloud2```

- ```segmentation``` - ```sensor_msgs/msg/Image```

  Must be an image of type MONO8 where the pixel value is the segmentation class.

- ```calibration ``` - ```sensor_msgs/msg/CameraInfo```

#### Publishes to

- ```pointcloud_filtered``` - ```sensor_msgs/msg/PointCloud2```

#### Parameters

- ```classes``` - ```int[]```, default ```{0}```

  The segmentation classes to re-publish.


### ```cbr::PclImageOverlayNode```

Projects a pointcloud onto an image as drawn circles and re-publishes the image. Useful for calibration verification.

Point colors are determined by the distance from the camera.

#### Subscribes to

- ```pointcloud``` - ```sensor_msgs/msg/PointCloud2```

- ```image``` - ```sensor_msgs/msg/Image```

  Must be an image of type RGB8

- ```calibration ``` - ```sensor_msgs/msg/CameraInfo```

#### Publishes to

- ```image_overlay``` - ```sensor_msgs/msg/Image```

#### Parameters

- ```max_size``` - ```int```, default ```10000```
 
  Maximal number of points to collect while waiting for the next image

- ```cmap_min, cmap_max``` - ```double```, default 2 and 20

  Saturation values for coloring

- ```draw_radius``` - ```int```, default ```2```

  Circle size
>>>>>>> Pcl image overlay
