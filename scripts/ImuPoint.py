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
    def send_transform(self):
        self.broadcaster.sendTransform(self.transform)

    def get_transform(self):
        pass

    def publish_msg(self):
        pass

    def reset(self):
        rospy.logerr("Resetting orientations")
        self.transform.transform.rotation = self.imu_msg.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [self.transform.transform.rotation.x,
             self.transform.transform.rotation.y,
             self.transform.transform.rotation.z,
             self.transform.transform.rotation.w])
        self.initYaw = yaw
        self.initPitch = pitch
        self.initRoll = roll
        rospy.logerr("Resetting " + self.name + " initial yaw: " + str(
            self.initYaw) + " pitch: " + str(self.initPitch) + ", roll: " + str(self.initRoll))
        self.set = True

    def imu_callback(self, msg):
        # rospy.logerr("In callback")
        self.imu_msg = msg

    def is_anchor(self):
        return self.is_anchor

    def print_(self):
        rospy.logerr(decor)
        rospy.logerr("Name: " + self.name)
        rospy.logerr("Parent Frame: " + self.parent)
        rospy.logerr("Child Frame: " + self.child)
        if self.is_anchor:
            rospy.logerr("Type: Anchor Point")
        else:
            rospy.logerr("Type: Imu Point")
            rospy.logerr("Listening to: " + self.sub_topic)

    def reset(self):
        self.initialTransform = self.transform

    def update(self):
        if not self.imu_msg:
            return

        if not self.initialTransform:
            self.initialTransform = TransformStamped()
            self.initialTransform.transform.rotation = self.imu_msg.orientation

        self.transform.transform.rotation = self.imu_msg.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [self.initialTransform.transform.rotation.x - self.transform.transform.rotation.x,
             self.initialTransform.transform.rotation.y - self.transform.transform.rotation.y,
             self.initialTransform.transform.rotation.z - self.transform.transform.rotation.z,
             self.initialTransform.transform.rotation.w - self.transform.transform.rotation.w])
        print(roll, pitch, yaw)
        if yaw == 0.0 or pitch == 0.0 or roll == 0.0:
            return
        if not self.set:
            self.initYaw = yaw
            self.initPitch = pitch
            self.initRoll = roll
            rospy.logerr(self.name + " initial yaw: " + str(
                self.initYaw) + " pitch: " + str(self.initPitch) + ", roll: " + str(self.initRoll))
            self.set = True

        if abs(self.roll - roll - self.initRoll) > 0.01:
            self.roll = roll - self.initRoll
        if abs(self.pitch - pitch - self.initPitch) > 0.01:
            self.pitch = pitch - self.initPitch
        if abs(self.yaw - yaw - self.initYaw) > 0.01:
            self.yaw = yaw - self.initYaw

        self.x = self.offset_x + self.distance_from_parent * math.cos(self.yaw) * math.cos(self.pitch)
        self.y = self.offset_y + self.distance_from_parent * math.cos(self.pitch) * math.sin(self.yaw)
        self.z = self.offset_z + self.distance_from_parent * math.sin(self.pitch)

        self.x2 = self.offset_x + (self.distance_from_parent + self.distance_from_child) * math.cos(
            self.yaw) * math.cos(self.pitch)
        self.y2 = self.offset_y + (self.distance_from_parent + self.distance_from_child) * math.cos(
            self.pitch) * math.sin(self.yaw)
        self.z2 = self.offset_z + (self.distance_from_parent + self.distance_from_child) * math.sin(self.pitch)
        #
        # self.x2 = self.offset_x * math.cos(self.yaw) * math.cos(self.pitch)
        # self.y2 = self.offset_y * math.cos(self.pitch) * math.sin(self.yaw)
        # self.z2 = self.offset_z * math.sin(self.pitch)

        self.transform.header.stamp = self.transform2.header.stamp = rospy.Time.now()

        q = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        msg = Quaternion(q[0], q[1], q[2], q[3])
        eq = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        empty = Quaternion(eq[0], eq[1], eq[2], eq[3])

        self.transform.header.frame_id = self.parent
        self.transform.child_frame_id = self.name
        self.transform.transform.rotation = msg
        self.transform.transform.translation.x = self.x
        self.transform.transform.translation.y = self.y
        self.transform.transform.translation.z = self.z

        self.transform2.header.frame_id = self.parent
        self.transform2.child_frame_id = self.child
        self.transform2.transform.rotation = empty
        self.transform2.transform.translation.x = self.x2
        self.transform2.transform.translation.y = self.y2
        self.transform2.transform.translation.z = self.z2

        # rospy.logerr("Publishing transforms")
        # self.broadcaster.sendTransform(self.transform)
        self.broadcaster.sendTransform(self.transform2)

    def __init__(self, parent_link_name="world", name="", params=None):
        if params is None:
            params = {}
        self.parent = parent_link_name
        if not name:
            raise ValueError("Trying to create anchor point with no name, exiting")
        self.name = name
        self.child = params["child_frame"]
        if not params:
            raise ValueError("Trying to create anchor point with no parameters, exiting")
        try:
            # Save parameters
            self.params = params
            self.broadcaster = tf2_ros.StaticTransformBroadcaster()
            self.imu_msg = Imu()
            # Create the initial transform
            self.transform = TransformStamped()
            self.transform2 = TransformStamped()
            self.initialTransform = []
            # Is the point an anchor?
            self.is_anchor = params["anchor"]
            if not self.is_anchor:
                # Listen for incoming imu msgs
                self.sub_topic = params["sub_topic"]
                self.subscriber = rospy.Subscriber(self.sub_topic, Imu, self.imu_callback, queue_size=1)

            self.offset_x = params["offset_x"]
            self.offset_y = params["offset_y"]
            self.offset_z = params["offset_z"]
            self.offset_qx = params["offset_qx"]
            self.offset_qy = params["offset_qy"]
            self.offset_qz = params["offset_qz"]
            self.offset_qw = params["offset_qw"]

            # Get Distances
            self.distance_from_parent = params["distance_from_parent"]
            self.distance_from_child = params["distance_from_child"]
            # Set up TF
            self.transform.header.frame_id = self.parent
            self.transform.child_frame_id = params["child_frame"]
            self.set = False
            self.prev_x = 0.0
            self.prev_y = 0.0
            self.prev_z = 0.0
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.x2 = 0.0
            self.y2 = 0.0
            self.z2 = 0.0
            self.yaw = 0.0
            self.pitch = 0.0
            self.roll = 0.0
            self.initYaw = 0.0
            self.initPitch = 0.0
            self.initRoll = 0.0
            rospy.logerr("Initializing: ")
            self.print_()
        except:
            rospy.logerr("Error while reading parameters:", sys.exc_info()[0])
            raise Exception("Error while reading parameters")


class ImuPointV2:
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
