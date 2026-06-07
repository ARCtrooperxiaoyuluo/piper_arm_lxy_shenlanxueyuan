#!/usr/bin/env python3
from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
#from piper_arm_PPO import PiperArm
from piper_arm import PiperArm
from utils.utils_piper import read_joints
from utils.utils_piper import enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from utils.utils_math import quaternion_to_rotation_matrix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

PI = math.pi
factor = 1000 * 180 / PI
receive_object_center = False
object_center = []
simulation = True


def control_arm(joints, speed=2):

    # joints [rad]

    position = joints

    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)

    if (joint_4 < -70000) :
        joint_4 = -70000

    # piper.MotionCtrl_1()
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    if len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    print(piper.GetArmStatus())
    print(position)

def object_point_callback(msg):
    # print("Receive visual detection result", msg.point.x, msg.point.y, msg.point.z)
    if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
        return
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]


def move_and_grasp(object_center, joints, piper_arm):
    print("prepare to grasp point under camera frame", object_center[0], object_center[1], object_center[2])

    # transfer point from camera frame to base_link frame
    base_T_link6 = piper_arm.forward_kinematics(joints)
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera

    base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])

    # publish target object center
    print("point under base frame", base_ob_center)
    pub = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
    publish_sphere_marker(pub, base_ob_center, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)

    targetT = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
    targetT[0, 3] = base_ob_center[0]
    targetT[1, 3] = base_ob_center[1]
    targetT[2, 3] = base_ob_center[2]


    # inverse kinematics
    joints = piper_arm.inverse_kinematics(targetT)
    joints_array = np.array(joints)
    print("base ob center", base_ob_center)
    if not joints :
        print("ik fail")
        return False
    print("Planed ik[degree]:", joints_array / PI * 180)

    # time_now = rospy.Time.now()
    # publish_tf(piper_arm, joints, time_now)

    joints.append(0.10)
    control_arm(joints, 20)
    time.sleep(10)
    # close gripper
    joints[6] = 0.02
    control_arm(joints, 20)
    time.sleep(2)
    # go back
    joints = [0, 0, -0.4, 0, 0, 0, 0.02]
    control_arm(joints, 20)

    return True



if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(0, 1000, 0x01, 0)

    # 设置初始位置
    joints = [0, 0, 0, 0, 0, 0, 0]
    control_arm(joints, 100)
    time.sleep(2)


    # 初始化节点
    rospy.init_node('vison_grasp_node', anonymous=True)

    piper_arm = PiperArm()
    sub = rospy.Subscriber('/object_point',
                           PointStamped,
                           object_point_callback,
                           queue_size=10,
                           tcp_nodelay=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # time_now = rospy.Time.now()
        # publish_tf(piper_arm, joints, time_now)
        if (receive_object_center):
            msg = piper.GetArmJointMsgs()

            theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
            theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
            theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
            theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
            theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
            theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

            joints = [theta1, theta2, theta3, theta4, theta5, theta6]

            if move_and_grasp(object_center, joints, piper_arm):
                break
            receive_object_center = False

        rate.sleep()







