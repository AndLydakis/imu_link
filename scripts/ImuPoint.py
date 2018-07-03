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

    def update(self):
        # rospy.logerr("Updating: " + self.name)
        if self.is_anchor:
            # self.broadcaster.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, msg.theta),
            #                                rospy.Time.now(), self.name, self.parent)
            return
        if not self.imu_msg:
            return
        self.transform.transform.rotation = self.imu_msg.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [self.transform.transform.rotation.x,
             self.transform.transform.rotation.y,
             self.transform.transform.rotation.z,
             self.transform.transform.rotation.w])
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
        self.z = - self.offset_z + self.distance_from_parent * math.sin(self.pitch)

        self.x2 = self.offset_x + (self.distance_from_parent + self.distance_from_child) * math.cos(self.yaw) * math.cos(self.pitch)
        self.y2 = self.offset_y + (self.distance_from_parent + self.distance_from_child) * math.cos(self.pitch) * math.sin(self.yaw)
        self.z2 = - self.offset_z + (self.distance_from_parent + self.distance_from_child) * math.sin(self.pitch)

        self.transform.header.stamp = self.transform2.header.stamp = rospy.Time.now()

        q = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        msg = Quaternion(q[0], q[1], q[2], q[3])
        eq = tf.transformations.quaternion_from_euler(0, 0, 0)
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
        self.broadcaster.sendTransform(self.transform)
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
