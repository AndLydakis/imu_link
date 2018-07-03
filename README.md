### imu_link

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

This file describes an anchor point located in the hip of a person, and two IMUs, located in mid thigh and mid tibia 
respectively. The resulting tree is

```html
<div>
<span style="color:black">world</span> -> <span style="color:red">pelvis</span> -> <span style="color:black">hip</span> ->
<span style="color:green">thigh</span> -> <span style="color:black">knee</span> -> <span style="color:green">tibia</span> 
-> <span style="color:black">ankle</span>
</div> 
```

Where red nodes are anchor points, green nodes are IMU points and black nodes are mostly decorative TFs for visualization.
