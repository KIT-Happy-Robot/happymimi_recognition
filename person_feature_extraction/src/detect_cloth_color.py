#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
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
        color = ''
        if 0 <= v and v <= 120: color = 'Black'
        elif (0 <= s and s <= 50) and (230 <= v and v <= 255): color = 'White'
        elif (0 <= s and s <= 50) and (120 <= v and v <= 150): color = 'Gray'
        elif (50 <= s and s <= 170) and (120 <= v and v <= 150): color = 'Gray'
        elif (0 <= s and s <= 50) and (120 <= v and v <= 230): color = 'Gray'
        elif (0 <= h and h <= 9) or (170 <= h and h <= 180): color = 'Red'
        elif 10 <= h and h <= 24: color = 'Orange'
        elif 25 <= h and h <= 39: color = 'Yellow'
        elif 40 <= h and h <= 89: color = 'Green'
        elif 90 <= h and h <= 136: color = 'Blue'
        elif 137<= h and h <= 159: color = 'Purple'
        elif 160 <= h and h <= 169: color = 'Pink'
        return color

    def main(self, _):
        color = SetStrResponse()

        self.head_pub.publish(-25.0)
        rospy.sleep(1.0)

        pose = self.pose_res
        if len(pose.persons)==0: return color

        # neckとhipの座標から中点を得る
        neck_x = pose.persons[0].bodyParts[1].pixel.y
        neck_y = pose.persons[0].bodyParts[1].pixel.x
        hip_x = pose.persons[0].bodyParts[15].pixel.y
        hip_y = pose.persons[0].bodyParts[15].pixel.x
        print 'neck: ', neck_x, neck_y
        print 'hip: ', hip_x, hip_y
        if (neck_x==0.0 and neck_y==0.0) and (hip_x==0.0 and hip_y==0.0):
            return color
        elif neck_x==0.0 and neck_y==0.0:
            center_x = hip_x
            center_y = hip_y + 1###
        elif hip_x==0.0 and hip_y==0.0:
            center_x = neck_x
            center_y = neck_y - 1###
        else:
            center_x = (nech_x+hip_x)/2
            center_y = (nech_y+hip_y)/2
        if center_y<0: center_y=0
        if center_y>480: center_y=480

        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color.result = judgeColor(hsv_image[int(center_x), int(center_y)])
        return color

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()
