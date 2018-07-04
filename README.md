## imu_link

The purpose of this ROS package is to link multiple IMUs to a mocap-like configuration.
Nodes are separated to ***anchor points*** and ***imu points***. 

***Anchor points*** are 
fixed points in the world, that can be used to provide the global coordinates of the ***imu points***.

***Imu points*** represent IMUs located in some part of your configuration (for example on an person's arm).
The first of these must be connected to an ***anchor point*** in the TF tree. IMUs start
with a known distance from the neighboring frames and an initial orientation with 
regards to their parent frame.

Each one subscribes to a ```sensor_msgs/Imu``` topic, and uses the published quaternions
to determine its position and orientation with regards to the parent frame.

The configurations are described in ```.yaml``` files. Below is an example configuration:

```yaml
anchor0:
  sub_topic: ""
  parent_frame: world
  child_frame: hip
  name: pelvis
  anchor: 1
  distance_from_parent: 0.15
  distance_from_child: 0.15
  offset_x: 0.0
  offset_y: -0.15
  offset_z: 0.0
  offset_qx: 0.0
  offset_qy: 0.0
  offset_qz: 0.0
  offset_qw: 0.0

joint0:
  sub_topic: imu
  parent_frame: hip
  child_frame: knee
  name: thigh
  anchor: 0
  distance_from_parent: 0.3
  distance_from_child: 0.3
  offset_x: 0.0
  offset_y: -0.15
  offset_z: 0.0
  offset_qx: 0.0
  offset_qy: 0.0
  offset_qz: 0.0
  offset_qw: 0.0

joint1:
  sub_topic: imu/data
  parent_frame: knee
  child_frame: ankle
  name: tibia
  anchor: 0
  distance_from_parent: 0.3
  distance_from_child: 0.3
  offset_x: 0.0
  offset_y: -0.15
  offset_z: 0.0
  offset_qx: 0.0
  offset_qy: 0.0
  offset_qz: 0.0
  offset_qw: 0.0
```

This file describes an anchor point located in the hip of a person, and two IMUs, located in mid thigh and mid tibia.
 The resulting tree is:

> world -> {- pelvis -} -> hip -> {+ thigh +} -> knee -> {+ tibia +} -> ankle

Where red nodes are anchor points, green nodes are IMU points and black nodes are mostly decorative TFs for visualization.

Below is a description of the different parameters for each point:
- **sub_topic:** The topic that the corresponding IMU publishes to (not needed for anchor points)
- **parent_frame:** The frame that acts as the parent frame for the ```parent_frame -> name``` transform
- **child_frame:** The frame that acts as the child frame for the ```name -> child_frame``` transform
- **name:** The name of the IMU/Anchor point, located between the parent and child frame.
- **anchor:** 1 if the point is an anchor point,  0 otherwise
- **distance_from_parent:** The straight line distance between ```parent_frame``` and the point 
- **distance_from_child:** The straight line distance between ```child_frame``` and the point 
- **offset_x/y/z:** The starting x,y,z offset between ```parent_frame``` and the point 
- **offset_qx/qy/qz/qw:** The starting orientation (as a quaternion [qx, qy, qz, qw]) offset between ```parent_frame``` and the point
 
### Execution
The sample launch file launches a Phidget IMU and a Razor 9Dof IMU in the configuration described above
```bash
user@host:~$ roslaunch imu_link imu_chain_publisher.launch
```
### Dependencies
No specific packages required, as long as you have ```sensor_msgs/Imu``` messages published from some other node

Tested on:
- ROS Kinetic
- Ubuntu 16.04

### TODO

### Project Members

- [Andreas Lydakis](andlydakis@gmail.com)