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
from playsound import playsound
import os


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

    # 比赛时需要注释掉
    # 绘制出aruco码的外框
    # aruco.drawDetectedMarkers(frame, corners, ids)
    # cv2.imshow("frame", frame)

    if ids == 0:
        # 设定目标点
        # target1 = Pose(Point(0.963, -1.038, 0.000), Quaternion(0.000, 0.000, 0.5746, 0.8184))
        rate2 = rospy.Rate(1)
        playsound('1veg.wav')
    elif ids == 1:
        # target1 = Pose(Point(0.4685, -1.0435, 0.000), Quaternion(0.000, 0.000, 0.698, 0.715))
        rate2 = rospy.Rate(1)
        playsound('2fruit.wav')
    elif ids == 2:
        # target1 = Pose(Point(-0.036, -1.125, 0.000), Quaternion(0.000, 0.000, 0.688, 0.72))
        rate2 = rospy.Rate(1)
        playsound('3meat.wav')
    

    # 按键1关闭程序
    # key = cv2.waitKey(1)
    while(1):
        rate2.sleep()


def aruco_demo():
    # 创立节点
    rospy.init_node('aruco_demo', anonymous=True)

    # 订阅usb_cam发出的图像消息，接收到消息后进入回调函数callback()
    rospy.Subscriber('camera/image_raw', Image, callback)

    rospy.sleep(3)
    # rospy.spin()
    rospy.wait_for_message("camera/image_raw", Image, timeout=None)
    sys.exit(0)
   

if __name__ == '__main__':
    aruco_demo()
    sys.exit()

    
    