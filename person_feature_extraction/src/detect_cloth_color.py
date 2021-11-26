#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import cv2
import collections
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from cv_bridge import CvBridge, CvBridgeError
from happymimi_msgs.srv import SetStr, SetStrResponse

class DetectClothColor(object):
    def __init__(self):
        rospy.Service('/person_feature/cloth_color', SetStr, self.main)
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
        h, s, v = req
        print h, s, v
        color = ''
        if 0<=v and v<=79: color = 'Black'
        elif (0<=s and s<=50) and (190<=v and v<=255): color = 'White'
        elif (0<=s and s<=50) and (80<=v and v<=130): color = 'Gray'
        #elif (50 <= s and s <= 170) and (70 <= v and v <= 150): color = 'Gray'
        elif (50<=s and s<=170) and (80<=v and v<=90): color = 'Gray'
        #elif (0<=s and s<=50) and (80<=v and v<=230): color = 'Gray'
        #elif (5<=h and h<=18) and (20<=s and s<=240) and (70<=v and v<=180): color = 'Brown'
        elif (5<=h and h<=18) and v<=200: color = 'Brown'
        elif (0<=h and h<=4) or (170<=h and h<=180): color = 'Red'
        elif 5<=h and h<=18: color = 'Orange'
        elif 19<=h and h<=39: color = 'Yellow'
        elif 40<=h and h<=89: color = 'Green'
        elif 90<=h and h<=136: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 160<=h and h<=169: color = 'Pink'
        return color

    def main(self, _):
        response = SetStrResponse()

        self.head_pub.publish(-20.0)
        rospy.sleep(2.5)

        pose = self.pose_res
        if len(pose.persons)==0: return response

        # neckとhipの座標から中点を得る
        neck_x = pose.persons[0].bodyParts[1].pixel.y
        neck_y = pose.persons[0].bodyParts[1].pixel.x
        hip_x = pose.persons[0].bodyParts[8].pixel.y
        hip_y = pose.persons[0].bodyParts[8].pixel.x
        print 'neck: ', neck_x, neck_y
        print 'hip: ', hip_x, hip_y
        if (neck_x==0.0 and neck_y==0.0) and (hip_x==0.0 and hip_y==0.0):
            return response
        elif neck_x==0.0 and neck_y==0.0:
            center_x = hip_x-10
            center_y = hip_y
        elif hip_x==0.0 and hip_y==0.0:
            center_x = neck_x+10
            center_y = neck_y
        else:
            center_x = (neck_x+hip_x)/2
            center_y = (neck_y+hip_y)/2
        if center_x<0: center_x=0
        if center_x>479: center_x=479
        if center_y<0: center_y=0
        if center_y>639: center_y=639

        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        #hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        color_map = []
        for i in range(-4, 5):
            x = center_x + i
            if x<0 or x>479: continue
            for j in range(-4, 5):
                y = center_y + j
                if y<0 or y>639: continue
                color = self.judgeColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print color_map
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]

        return response

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()
