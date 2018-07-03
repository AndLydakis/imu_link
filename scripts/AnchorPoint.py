#!/usr/bin/env python

import math
import sys
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu


class AnchorPoint:
    def send_transform(self):
        self.broadcaster.sendTransform(self.transform)

    def publish_msg(self):
        pass

    def reset(self):
        pass

    def imu_callback(self, msg):
        pass

    def __init__(self, parent_link_name="world", name="", params=None):
        if params is None:
            params = {}
        self.parent = parent_link_name
        if not name:
            raise ValueError("Trying to create anchor point with no name, exiting")
        self.name = name
        if not params:
            raise ValueError("Trying to create anchor point with no parameters, exiting")
        try:
            # Save parameters
            self.params = params
            self.broadcaster = tf2_ros.StaticTransformBroadcaster()
            # Listen for incoming imu msgs
            self.sub_topic = params["sub_topic"]
            self.subscriber = rospy.Subscriber(self.sub_topic, Imu, self.imu_callback, queue_size=10)
            self.imu_msg = Imu()
            # Create the initial transform
            self.transform = TransformStamped()
            self.transform.header.frame_id = self.parent
            self.transform.child_frame_id = params["child_frame"]
            # self.transform.transform.translation.x = params["x"]
            # self.transform.transform.translation.y = params["y"]
            # self.transform.transform.translation.z = params["z"]
            # self.transform.transform.rotation.x = params["qx"]
            # self.transform.transform.rotation.y = params["qy"]
            # self.transform.transform.rotation.z = params["qz"]
            # self.transform.transform.rotation.w = params["qw"]
            self.prev_x = 0.0
            self.prev_y = 0.0
            self.prev_z = 0.0
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.yaw = 0.0
            self.pitch = 0.0
            self.roll = 0.0
            self.initYaw = 0.0
            self.initPitch = 0.0
            self.initRoll = 0.0
        except:
            print "Error while reading parameters:", sys.exc_info()[0]
