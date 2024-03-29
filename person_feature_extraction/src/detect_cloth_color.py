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
import mimi_color

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
        r_shoulder_x = pose.persons[0].bodyParts[2].pixel.y
        r_shoulder_y = pose.persons[0].bodyParts[2].pixel.x
        l_shoulder_x = pose.persons[0].bodyParts[5].pixel.y
        l_shoulder_y = pose.persons[0].bodyParts[5].pixel.x
        print('neck: ', neck_x, neck_y)
        print('hip: ', hip_x, hip_y)
        print('r_shoulder: ', r_shoulder_x, r_shoulder_y)
        print('l_shoulder: ', l_shoulder_x, l_shoulder_y)

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

        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']

        disColor = mimi_color.DetectColor()

        for i in range(chest_length+1):
            x = body_axis_x + i
            #if x<0 or x>479: continue
            for j in range(-width, width):
                y = body_axis_y + j
                #if y<0 or y>639: continue
                color = disColor.detectColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print(color_map)
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]

        return response

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()
