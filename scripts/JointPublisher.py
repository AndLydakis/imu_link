#!/usr/bin/env python


import rospy
from ImuPoint import ImuPoint

decor = "#----------------------------------------------#"


def read_joints_from_params():
    joints = {}
    count = 0
    rospy.loginfo(rospy.has_param("/joint" + str(count) + "/name"))
    while True:
        if rospy.has_param("/joint" + str(count) + "/name"):
            params = {"sub_topic": rospy.get_param("/joint" + str(count) + "/sub_topic"),
                      "parent_frame": rospy.get_param("/joint" + str(count) + "/parent_frame"),
                      "child_frame": rospy.get_param("/joint" + str(count) + "/child_frame"),
                      "name": rospy.get_param("/joint" + str(count) + "/name"),
                      "offset_x": float(rospy.get_param("/joint" + str(count) + "/offset_x")),
                      "offset_y": float(rospy.get_param("/joint" + str(count) + "/offset_y")),
                      "offset_z": float(rospy.get_param("/joint" + str(count) + "/offset_z")),
                      "offset_roll": float(rospy.get_param("/joint" + str(count) + "/offset_roll")),
                      "offset_pitch": float(rospy.get_param("/joint" + str(count) + "/offset_pitch")),
                      "offset_yaw": float(rospy.get_param("/joint" + str(count) + "/offset_yaw"))}
            print(params)
            joints[params["name"]] = ImuPoint(name_=params["name"], child_frame_=params["child_frame"],
                                              parent_frame_=params["parent_frame"], sub_topic_=params["sub_topic"],
                                              ipitch_=params["offset_pitch"], iroll_=params["offset_roll"],
                                              iyaw_=params["offset_yaw"], ix_=params["offset_x"],
                                              iy_=params["offset_y"], iz_=params["offset_z"])
            count += 1
        else:
            break

    rospy.loginfo(decor)
    rospy.loginfo("Found " + str(count) + " joints")
    rospy.loginfo(joints)
    rospy.loginfo(decor)
    return joints


class JointPublisher:

    def __init__(self):
        rospy.init_node("imu_joint_publisher")
        self.r = rospy.Rate(5)
        self.joints = read_joints_from_params()
        if not self.joints:
            rospy.logerr("No joints detected, exiting")
            exit(-1)

    def run(self):
        rospy.logerr("Joint Publisher Spinning")
        while not rospy.is_shutdown():
            for joint_id, joint in self.joints.iteritems():
                joint.update()
            self.r.sleep()


if __name__ == "__main__":
    jp = JointPublisher()
    jp.run()
