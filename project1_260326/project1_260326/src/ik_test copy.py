#!/usr/bin/env python3

from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_piper import read_joints
from utils.utils_piper import enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

PI = math.pi
factor = 1000 * 180 / PI
def test_single_point(armIK):
    targetT = np.eye(4)
    targetT[0, 3] = 0.078
    targetT[2, 3] = 0.39581  #[0.0, 0.7965034341280486, -0.7972333510532297, 0, 0, 0]

    # targetT[0, 3] = 0.30884
    # targetT[2, 3] = 0.014427

    # targetT[0, 3] = -0.125713
    # targetT[2, 3] = 0.435287      #[-0.0, 0.2342462279662385, -1.0405477710374524, 0, 0, 0]

    joints = armIK.inverse_kinematics(targetT)
    print(joints)

    # control_arm(joints)

def test_circle_trajectory(armIK):
    cnt = 0
    pub = rospy.Publisher('/ik_traget_point', Marker, queue_size=10)
    path_pub = rospy.Publisher('/planned_trajectory', Path, queue_size=10)

    while cnt < 10000 and (not rospy.is_shutdown()):
        # define target pose
        x = 0.1 * math.sin(cnt / 500 * PI) + 0.4
        y = 0.1 * math.cos(cnt / 500 * PI)
        z = 0.45
        cnt += 1

        targetT = np.array([[0, 0, 1, 0],[0, 1, 0, 0],[-1, 0, 0, 0],[0, 0, 0, 1]],dtype=float)
        targetT[0, 3] = x
        targetT[1, 3] = y
        targetT[2, 3] = z

        # publish marker
        point = np.array([x, y, z])
        publish_sphere_marker(pub, point, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)
        # publish_sphere_marker(pub, point, frame_id="base_link", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)
        publish_trajectory(path_pub, point)

        # inverse kinematics
        joints = armIK.inverse_kinematics(targetT)
        print(joints)

        time_now = rospy.Time.now()
        publish_tf(armIK, joints, time_now)
        control_arm(joints)

        # publish tf from IK
        time.sleep(0.01)

def test_sqr_trajectory(armIK):
    cnt = 0
    pub = rospy.Publisher('/ik_traget_point', Marker, queue_size=10)
    path_pub = rospy.Publisher('/planned_trajectory', Path, queue_size=10)

    while cnt < 10000 and (not rospy.is_shutdown()):
        # define target pose
        # 方形轨迹：边长 = 圆形直径(0.2)，中心点 (0.4, 0)，和原运动周期一致
        t = (cnt / 150) % 4.0  # 0~3 对应方形4条边
        side = 0.1  # 半边长 = 原半径，总边长=0.2

        if t < 1.0:
            # 边1：右 → 上
            x = 0.4 + side
            y = -side + 2 * side * t
        elif t < 2.0:
            # 边2：上 → 左
            x = 0.4 + side - 2 * side * (t - 1.0)
            y = side
        elif t < 3.0:
            # 边3：左 → 下
            x = 0.4 - side
            y = side - 2 * side * (t - 2.0)
        else:
            # 边4：下 → 右
            x = 0.4 - side + 2 * side * (t - 3.0)
            y = -side
        z = 0.45
        cnt += 1

        targetT = np.array([[0, 0, 1, 0],[0, 1, 0, 0],[-1, 0, 0, 0],[0, 0, 0, 1]],dtype=float)
        targetT[0, 3] = x
        targetT[1, 3] = y
        targetT[2, 3] = z

        # publish marker
        point = np.array([x, y, z])
        publish_sphere_marker(pub, point, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)
        # publish_sphere_marker(pub, point, frame_id="base_link", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)
        publish_trajectory(path_pub, point)

        # inverse kinematics
        joints = armIK.inverse_kinematics(targetT)
        print(joints)

        time_now = rospy.Time.now()
        publish_tf(armIK, joints, time_now)
        control_arm(joints)

        # publish tf from IK
        time.sleep(0.01)


def control_arm(joints):

    position = joints

    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)
    # joint_6 = round(position[6] * 1000 * 1000)
    # piper.MotionCtrl_1()
    piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    # piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    # print(piper.GetArmStatus())
    # print(position)

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(0, 1000, 0x01, 0)

    # 初始化节点
    rospy.init_node('ik_node', anonymous=True)

    armIK = PiperArm()
    # test_single_point(armIK)
    # sys.exit()

    test_sqr_trajectory(armIK)
    # test_circle_trajectory(armIK)

    # publish TF
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        joints = read_joints(piper)
        time_now = rospy.Time.now()
        publish_tf(armIK, joints, time_now)
        rate.sleep()





