#!/usr/bin/env python

import math
import rospy
import tf
import tf2_ros
from AnchorPoint import AnchorPoint
from ImuPoint import ImuPoint
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped

decor = "#----------------------------------------------#"


class JointPublisher:

    def __init__(self):
        rospy.init_node("imu_joint_publisher")
        r = rospy.Rate(100)
        anchors = {}
        joints = {}
        count = 0
        while True:
            try:
                if rospy.has_param("/anchor" + str(count) + "/name"):
                    params = {}
                    params["sub_topic"] = rospy.get_param("/anchor" + str(count) + "/sub_topic")
                    params["parent_frame"] = rospy.get_param("/anchor" + str(count) + "/parent_frame")
                    params["child_frame"] = rospy.get_param("/anchor" + str(count) + "/child_frame")
                    params["name"] = rospy.get_param("/anchor" + str(count) + "/name")
                    params["anchor"] = int(rospy.get_param("/anchor" + str(count) + "/anchor"))
                    params["distance_from_parent"] = float(
                        rospy.get_param("/anchor" + str(count) + "/distance_from_parent"))
                    params["distance_from_child"] = float(
                        rospy.get_param("/anchor" + str(count) + "/distance_from_child"))

                    params["offset_x"] = float(rospy.get_param("/anchor" + str(count) + "/offset_x"))
                    params["offset_y"] = float(rospy.get_param("/anchor" + str(count) + "/offset_y"))
                    params["offset_z"] = float(rospy.get_param("/anchor" + str(count) + "/offset_z"))

                    params["offset_qx"] = float(rospy.get_param("/anchor" + str(count) + "/offset_qx"))
                    params["offset_qy"] = float(rospy.get_param("/anchor" + str(count) + "/offset_qy"))
                    params["offset_qz"] = float(rospy.get_param("/anchor" + str(count) + "/offset_qz"))
                    params["offset_qw"] = float(rospy.get_param("/anchor" + str(count) + "/offset_qw"))

                    anchors[params["name"]] = ImuPoint(params["parent_frame"], params["name"], params)
                    count += 1
                else:
                    break
            except:
                break
        rospy.loginfo(decor)
        rospy.loginfo("Found " + str(count) + " anchors")
        rospy.loginfo(anchors)
        count = 0
        while True:
            try:
                if rospy.has_param("/joint" + str(count) + "/name"):
                    params = {}
                    params["sub_topic"] = rospy.get_param("/joint" + str(count) + "/sub_topic")
                    params["parent_frame"] = rospy.get_param("/joint" + str(count) + "/parent_frame")
                    params["child_frame"] = rospy.get_param("/joint" + str(count) + "/child_frame")
                    params["name"] = rospy.get_param("/joint" + str(count) + "/name")
                    params["anchor"] = int(rospy.get_param("/joint" + str(count) + "/anchor"))
                    params["distance_from_parent"] = float(
                        rospy.get_param("/joint" + str(count) + "/distance_from_parent"))
                    params["distance_from_child"] = float(
                        rospy.get_param("/joint" + str(count) + "/distance_from_child"))

                    params["offset_x"] = float(rospy.get_param("/joint" + str(count) + "/offset_x"))
                    params["offset_y"] = float(rospy.get_param("/joint" + str(count) + "/offset_y"))
                    params["offset_z"] = float(rospy.get_param("/joint" + str(count) + "/offset_z"))

                    params["offset_qx"] = float(rospy.get_param("/joint" + str(count) + "/offset_qx"))
                    params["offset_qy"] = float(rospy.get_param("/joint" + str(count) + "/offset_qy"))
                    params["offset_qz"] = float(rospy.get_param("/joint" + str(count) + "/offset_qz"))
                    params["offset_qw"] = float(rospy.get_param("/joint" + str(count) + "/offset_qw"))
                    
                    joints[count] = ImuPoint(params["parent_frame"], params["name"], params)
                    rospy.loginfo(params)
                    count += 1
                    print(params)
                else:
                    rospy.loginfo("Done reading parameters")
                    break
            except:
                break
        rospy.loginfo("Found " + str(count) + " joints")
        rospy.loginfo(joints)
        rospy.loginfo(decor)
        while not rospy.is_shutdown():
            for anchor_id, anchor in anchors.iteritems():
                anchor.update()
            for joint_id, joint in joints.iteritems():
                joint.update()
            r.sleep()


if __name__ == "__main__":
    JointPublisher()
