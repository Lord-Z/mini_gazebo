#! /usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
import time
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import roslib
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys
import os
from playsound import playsound


def callback(data):
    # 通过cvBridge将ROS中图片的格式转换为Opencv中的格式
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # 轴对称翻转照片，否则二维码是镜像的
    # frame = cv2.flip(frame,1)

    # 将图像转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 加载aruco字典，本次比赛使用的是4x4的aruco码
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

    # 建立aruco检测参数，默认即可
    parameters = aruco.DetectorParameters_create()

    # 检测aruco码的角点信息
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # 如果检测到aruco码，输出其编号
    if ids is not None:
        print(ids)

    # # 比赛时需要注释掉
    # # 绘制出aruco码的外框
    # aruco.drawDetectedMarkers(frame, corners, ids)
    # cv2.imshow("frame", frame)

    # 按键1关闭程序
    #key = cv2.waitKey(1)

    # 修改部分，返回id值
    move(ids)


def move(ids):

    # 订阅move_base服务器的消息
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    ids = 0
    # 设定目标点
    if ids == 0:
        target1 = Pose(Point(1.0121, -1.0124, 0.000), Quaternion(0.000, 0.000, 0.5746, 0.8184))
        #target1 = Pose(Point(1.1094, --0.99246, 0.000), Quaternion(0.000, 0.000, 0.5746, 0.8184))
        #target1 = Pose(Point(0.963, -1.038, 0.000), Quaternion(0.000, 0.000, 0.5746, 0.8184))
    elif ids == 1:
        target1 = Pose(Point(0.4805, -1.0323, 0.000), Quaternion(0.000, 0.000, 0.698, 0.715))
        #target1 = Pose(Point(0.4890, -1.0435, 0.000), Quaternion(0.000, 0.000, 0.698, 0.715))
        #target1 = Pose(Point(0.4685, -1.0435, 0.000), Quaternion(0.000, 0.000, 0.698, 0.715))
    elif ids == 2:
        target1 = Pose(Point(-0.0430, -1.0764, 0.000), Quaternion(0.000, 0.000, 0.688, 0.72))
        #target1 = Pose(Point(-0.03395, -0.93968, 0.000), Quaternion(0.000, 0.000, 0.688, 0.72))
        #target1 = Pose(Point(-0.036, -1.125, 0.000), Quaternion(0.000, 0.000, 0.688, 0.72))

    goal = MoveBaseGoal()
    goal.target_pose.pose = target1
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # os.system('python play{}.py'.format(ids))

    rospy.loginfo("Going to: " + str(target1))

    start_time = rospy.Time.now()

    # 向目标进发
    move_base.send_goal(goal)

    # 五分钟时间限制
    finished_within_time = move_base.wait_for_result(rospy.Duration(300))

    # 运行所用时间
    running_time = rospy.Time.now() - start_time
    running_time = running_time.secs

    # 查看是否成功到达
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo("Timed out achieving goal")
        
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
            playsound('goal2.wav')
            os._exit(0)
        else:
            rospy.loginfo("Goal failed！ ")


def my_move():

    rospy.init_node('move_test', anonymous=True)
    # 订阅move_base服务器的消息
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")

    # 等待连接服务器，5s等待时间限制
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move base server")

    # 先去设定的B点
    target = Pose(Point(2.2611, -2.70569, 0.000), Quaternion(0.000, 0.000, 0.99999, 0.002084))
    # target = Pose(Point(2.429, -2.861, 0.000), Quaternion(0.000, 0.000, 0.979, 0.202))
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    rospy.loginfo("Going to: " + str(target))

    start_time = rospy.Time.now()

    # 向目标进发
    move_base.send_goal(goal)

    # 五分钟时间限制
    finished_within_time = move_base.wait_for_result(rospy.Duration(300))

    # 运行所用时间
    running_time = rospy.Time.now() - start_time
    running_time = running_time.secs

    # 查看是否成功到达
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo("Timed out achieving goal")
    else:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
        else:
            rospy.loginfo("Goal failed！ ")

    os.system('python learn.py')
    

    rate = rospy.Rate(1)

    rospy.Subscriber('camera/image_raw', Image, callback)
    
    # 等待
    rospy.sleep(40)

    rospy.wait_for_message("camera/image_raw", Image, timeout=None)


if __name__ == '__main__':
    my_move()
    
    