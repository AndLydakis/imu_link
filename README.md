## imu_link

The purpose of this ROS package is to link multiple IMUs to a mocap-like configuration based on offsets defined in
config files. 

Each ```ImuPoint``` subscribes to a ```sensor_msgs/Imu``` topic, and uses the published quaternions
to determine its position and orientation with regards to the parent frame.

The configurations are described in ```.yaml``` files. Below is an example configuration:

```yaml
joint0:
  sub_topic: /imu/data
  parent_frame: world
  name: hip
  child_frame: knee
  offset_x: 0.0
  offset_y: 0.0
  offset_z: -0.30
  offset_roll: 0.0
  offset_pitch: 0.0
  offset_yaw: 0.0

joint1:
  sub_topic: /imu/
  parent_frame: knee
  name: knee2
  child_frame: ankle
  offset_x: 0.0
  offset_y: 0.0
  offset_z: -0.30
  offset_roll: 0.0
  offset_pitch: 0.0
  offset_yaw: 0.0
```

This file describes a configuration for two IMUs located on the hip and knee joint:

world -> 
```diff
+ hip
```
-> knee ->
```diff
+ knee2
```
-> ankle

Where green nodes are IMU points and black nodes are mostly decorative TFs for visualization.

Below is a description of the different parameters for each point:
- **sub_topic:** The topic that the corresponding IMU publishes to (not needed for anchor points)
- **parent_frame:** The frame that acts as the parent frame for the ```parent_frame -> name``` transform
- **child_frame:** The frame that acts as the child frame for the ```name -> child_frame``` transform
- **name:** The name of the IMU/Anchor point, located between the parent and child frame.
- **offset_x/y/z:** The starting x,y,z offset between ```name``` and ```child_frame```
- **offset_roll/pitch/yaw:** The starting orientation (as a quaternion [qx, qy, qz, qw]) offset between ```name``` and ```child_frame```
 
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
