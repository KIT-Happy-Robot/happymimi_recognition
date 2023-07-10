#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import cv2
import collections
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib as plt
import mimi_color

from happymimi_msgs.srv import SetStr, SetStrResponse

class DetectClothColor(object):
    def __init__(self):
        rospy.Service('/person_feature/hair_color', SetStr, self.main)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.image_res = Image()
        self.pose_res = Frame()

    def realsenseCB(self, res):
        self.image_res = res

    def openPoseCB(self, res):
        self.pose_res = res
        
    def main(self, _):
        response = SetStrResponse()

        self.head_pub.publish(-20.0)
        rospy.sleep(2.5)

        pose = self.pose_res
        if len(pose.persons)==0: return response

        reye_x = pose.persons[0].bodyParts[15].pixel.y
        reye_y = pose.persons[0].bodyParts[15].pixel.x
        leye_x = pose.persons[0].bodyParts[16].pixel.y
        leye_y = pose.persons[0].bodyParts[16].pixel.x
        nose_x = pose.persons[0].bodyParts[0].pixel.y
        nose_y = pose.persons[0].bodyParts[0].pixel.x
        
        rear_x = pose.persons[0].bodyParts[17].pixel.y
        rear_y = pose.persons[0].bodyParts[17].pixel.x
        lear_x = pose.persons[0].bodyParts[18].pixel.y
        lear_y = pose.persons[0].bodyParts[17].pixel.x                           
        
        print('reye: ', reye_x, reye_y)
        print('leye: ', leye_x, leye_y)
        print('nose: ', nose_x, nose_y)

        width = int(reye_y - leye_y)
        face_axis_x = int(nose_x + 40)
        face_axis_y = int(nose_y + 40)
        face_length = 50
        if width < 0:
            width = -1*width
        
        print("face_length:", face_length)
        print("face_axis_x:", face_axis_x)
        print("face_axis_y:", face_axis_y)
        print("width:", width)
        
        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']

        disColor = mimi_color.DetectColor()

        for i in range(face_length):
            x = i 
            if 0<x and x>479:continue
            for j in range(width):
                y = j 
                if y>639:continue
                color = disColor.detectColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print(color_map)
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]
        
        print(response.result)
        return response

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()