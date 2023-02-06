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

    def judgeColor(self, req):
        # hsv色空間で色の判定
        s, h, v = req
        #print h, s, v
        color = ''
        if 0<=v and v<=79: color = 'Black'
        elif (25<=h and h<= 35) and (20<=s and s<=30): color = 'skin'
        elif (0<=s and s<=50) and (190<=v and v<=255): color = 'White'
        elif (0<=s and s<=50) and (80<=v and v<=130): color = 'Gray'
        #elif (50 <= s and s <= 170) and (70 <= v and v <= 150): color = 'Gray'
        #elif (50<=s and s<=170) and (80<=v and v<=90): color = 'Gray'
        #elif (0<=s and s<=50) and (80<=v and v<=230): color = 'Gray'
        #elif (5<=h and h<=18) and (20<=s and s<=240) and (70<=v and v<=180): color = 'Brown'
        elif (5<=h and h<=18) and v<=200: color = 'Brown'
        elif (0<=h and h<=4) or (174<=h and h<=180): color = 'Red'
        elif 5<=h and h<=18: color = 'Orange'
        elif 20<=h and h<=39: color = 'Yellow'
        elif 40<=h and h<=89: color = 'Green'
        elif 180<=h and h<=240: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 160<=h and h<=173: color = 'Pink'
        return color
        
    def main(self, _):
        response = SetStrResponse()

        self.head_pub.publish(-20.0)
        rospy.sleep(2.5)

        pose = self.pose_res
        if len(pose.persons)==0: return response

        # neckとhipの座標から中点を得る
        reye_x = pose.persons[0].bodyParts[15].pixel.x
        reye_y = pose.persons[0].bodyParts[15].pixel.y
        leye_x = pose.persons[0].bodyParts[16].pixel.x
        leye_y = pose.persons[0].bodyParts[16].pixel.y
        nose_x = pose.persons[0].bodyParts[0].pixel.x
        nose_y = pose.persons[0].bodyParts[0].pixel.y
        
        rear_x = pose.persons[0].bodyParts[17].pixel.x
        rear_y = pose.persons[0].bodyParts[17].pixel.y
        lear_x = pose.persons[0].bodyParts[18].pixel.x
        lear_y = pose.persons[0].bodyParts[17].pixel.y                           
        
        print('reye: ', reye_x, reye_y)
        print('leye: ', leye_x, leye_y)
        print('nose: ', nose_x, nose_y)

        width = int(leye_y - reye_y)
        face_axis_x = int(nose_x)
        face_axis_y = int(nose_y)
        face_length = 30

        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']
        for i in range(face_length+1):
            x = face_axis_x + i
            if x<0 or x>479: continue
            for j in range(-width, width):
                y = face_axis_y + j
                if y<0 or y>639: continue
                color = self.judgeColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print(color_map)
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]

        return response

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()