#!/usr/bin/env python3
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
        rospy.Service('/person_feature/pants_color', SetStr, self.main)
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
        elif (0<=s and s<=50) and (190<=v and v<=255): color = 'White'
        elif (0<=s and s<=50) and (80<=v and v<=130): color = 'Gray'
        #elif (50 <= s and s <= 170) and (70 <= v and v <= 150): color = 'Gray'
        elif (50<=s and s<=170) and (80<=v and v<=90): color = 'Gray'
        #elif (0<=s and s<=50) and (80<=v and v<=230): color = 'Gray'
        #elif (5<=h and h<=18) and (20<=s and s<=240) and (70<=v and v<=180): color = 'Brown'
        elif (5<=h and h<=18) and v<=200: color = 'Brown'
        elif (0<=h and h<=4) or (174<=h and h<=180): color = 'Red'
        elif 5<=h and h<=18: color = 'Orange'
        elif 19<=h and h<=39: color = 'Yellow'
        elif 40<=h and h<=89: color = 'Green'
        elif 90<=h and h<=136: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 160<=h and h<=173: color = 'Pink'
        return color

    def main(self, _):
        response = SetStrResponse()

        #ここを下側に向けたい
        #ズボンの色を検出したいから
        #hipとankleが見えればおけ
        self.head_pub.publish(-20.0)
        rospy.sleep(2.5)

        pose = self.pose_res
        if len(pose.persons)==0: return response

        # ankleとhipの座標を得る
        rhip_x = pose.persons[0].bodyParts[9].pixel.y
        rhip_y = pose.persons[0].bodyParts[9].pixel.x
        lhip_x = pose.persons[0].bodyParts[12].pixel.y
        lhip_y = pose.persons[0].bodyParts[12].pixel.x

        rankle_x = pose.persons[0].bodyParts[11].pixel.y
        rankle_y = pose.persons[0].bodyParts[11].pixel.x
        lankle_x = pose.persons[0].bodyParts[14].pixel.y
        lankle_y = pose.persons[0].bodyParts[14].pixel.x

        print("rhip_x",rhip_x)
        print("rhip_y",rhip_y)
        print("lhip_x",lhip_x)
        print("lhip_y",lhip_y)
        print("---------------------")
        print("rankle_x",rankle_x)
        print("rankle_y",rankle_y)
        print("lankle_x",rankle_x)
        print("lankle_y",rankle_y)
        '''
        if (neck_x==0.0 and neck_y==0.0) and (hip_x==0.0 and hip_y==0.0):
            return response
        elif neck_x==0.0 and neck_y==0.0:
            body_axis_x = hip_x-10
            body_axis_y = hip_y
            chest_length = 10
        else:
            body_axis_x = neck_x
            body_axis_y = neck_y
            if hip_x==0.0 and hip_y==0.0:
                chest_length = int(479 - neck_x)
            else:
                chest_length = int(hip_x - neck_x)
        if body_axis_x<0: body_axis_x=0
        if body_axis_x>479: body_axis_x=479
        if body_axis_y<0: body_axis_y=0
        if body_axis_y>639: body_axis_y=639

        if (r_shoulder_x==0.0 and r_shoulder_y==0.0) and (l_shoulder_x==0.0 and l_shoulder_y==0.0):
            pass
        elif r_shoulder_x==0.0 and r_shoulder_y==0.0:
            width = int(l_shoulder_y - body_axis_y)
        elif l_shoulder_x==0.0 and l_shoulder_y==0.0:
            width = int(body_axis_y - r_shoulder_y)
        else:
            shoulder_width = l_shoulder_y - r_shoulder_y
            width = int(shoulder_width/2)
        '''
        leg_length = int(rankle_x - rhip_x)
        leg_axis_x = int(rhip_x)
        leg_axis_y = int(rhip_y)
        width      = int(rhip_x - lhip_x)
        if width < 0:
            width = -1*width
        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']
        for i in range(leg_length):
            x = i
            if x>479: continue
            for j in range(width):
                y = j
                if y>639: continue
                color = self.judgeColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        #print(color_map)
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]

        print(response.result)
        return response

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()