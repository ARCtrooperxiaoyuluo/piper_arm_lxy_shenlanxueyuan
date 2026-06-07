#!/usr/bin/env python3

from piper_sdk import *
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
import time
import math
PI = math.pi

import sys
from piper_arm import PiperArm
import utils.utils_ros
from utils.utils_ros import publish_tf


if __name__ == "__main__":

    # 初始化节点
    rospy.init_node('fk_node', anonymous=True)
    # 创建发布者
    pub = rospy.Publisher('/fk_marker', Marker, queue_size=10)

    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    piper_arm = PiperArm()

    while not rospy.is_shutdown():

        print(piper.GetArmJointMsgs())
        msg = piper.GetArmJointMsgs()

        theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
        theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
        theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
        theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
        theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
        theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

        joints = [theta1, theta2, theta3, theta4, theta5, theta6]
        print("thetas", joints)

        time_now = rospy.Time.now()
        publish_tf(piper_arm, joints, time_now)

        T_total = piper_arm.forward_kinematics(joints)

        l6_point = np.array([0, 0, 0.1358])
        # l6_point = np.array([0, 0, 0])
        l6_point_homo = np.append(l6_point, 1.0)

        w_point = T_total @ l6_point_homo.T
        w_point = w_point[:3]

        print("w_point:", w_point)
        utils.utils_ros.publish_sphere_marker(pub, w_point)

        time.sleep(0.01)
        pass
