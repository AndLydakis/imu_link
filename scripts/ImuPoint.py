#!/usr/bin/env python

import math
import sys
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

decor = "#----------------------------------------------#"


class ImuPoint:
    def __init__(self, dfp_=0.0, dfc_=0.0, yaw_=0.0, pitch_=0.0, roll_=0.0, parent_frame_="", name_="", child_frame_="",
                 iroll_=0.0, ipitch_=0.0, iyaw_=0.0, ix_=0.0, iy_=0.0, iz_=0.0, sub_topic_="", anchor_=False, ):
        self.initial_transform = TransformStamped()
        self.static_transform = TransformStamped()
        self.current_transform = TransformStamped()
        self.child_transform = TransformStamped()
        self.imu_msg = Imu()
        self.distance_from_parent = dfp_
        self.distance_from_child = dfc_
        self.yaw = yaw_
        self.roll = pitch_
        self.pitch = roll_
        self.pframe = parent_frame_
        self.cframe = child_frame_
        self.frame_name = name_
        self.static_roll = math.radians(iroll_)
        self.static_pitch = math.radians(ipitch_)
        self.static_yaw = math.radians(iyaw_)
        self.init_x = ix_
        self.init_y = iy_
        self.init_z = iz_
        self.is_anchor = anchor_
        self.got_imu_msg = False
        q = tf.transformations.quaternion_from_euler(self.static_roll, self.static_pitch,
                                                     self.static_yaw)
        self.static_transform.transform.rotation.x = q[0]
        self.static_transform.transform.rotation.y = q[1]
        self.static_transform.transform.rotation.z = q[2]
        self.static_transform.transform.rotation.w = q[3]

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        if not self.is_anchor:
            # Listen for incoming imu msgs
            self.sub_topic = sub_topic_
            self.subscriber = rospy.Subscriber(self.sub_topic, Imu, self.imu_callback, queue_size=1)
            self.got_imu_msg = False
        else:
            self.got_imu_msg = True

    def reset(self):
        self.initial_transform.transform.rotation = self.imu_msg.orientation

    def update(self):
        if not self.got_imu_msg:
            return

        # Find angle between initial rotation current imu measurements
        q_cur = [self.imu_msg.orientation.x, self.imu_msg.orientation.y,
                 self.imu_msg.orientation.z, self.imu_msg.orientation.w]

        q_init_inv = [self.initial_transform.transform.rotation.x, self.initial_transform.transform.rotation.y,
                      self.initial_transform.transform.rotation.z, -self.initial_transform.transform.rotation.w]

        res = tf.transformations.quaternion_multiply(q_cur, q_init_inv)

        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion([res[0], res[1], res[2], res[3]])

        self.roll += self.static_roll
        self.pitch += self.static_pitch
        self.yaw += self.static_yaw

        # print("*", self.static_roll, self.static_roll, self.static_yaw)
        print(self.roll, self.pitch, self.yaw)

        q = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.current_transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])

        self.current_transform.header.frame_id = self.pframe
        self.current_transform.child_frame_id = self.frame_name

        self.broadcaster.sendTransform(self.current_transform)

        self.child_transform.header.frame_id = self.frame_name
        self.child_transform.child_frame_id = self.cframe
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.child_transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        self.child_transform.transform.translation.x = self.init_x
        self.child_transform.transform.translation.y = self.init_y
        self.child_transform.transform.translation.z = self.init_z
        self.broadcaster.sendTransform(self.child_transform)

    def imu_callback(self, msg):
        self.imu_msg = msg
        if not self.got_imu_msg:
            self.initial_transform.transform.rotation = self.imu_msg.orientation
            self.got_imu_msg = True
