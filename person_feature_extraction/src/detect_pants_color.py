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

class DetectPantsColor(object):
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
        h,s,v = req
        print(h, s, v)
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
        '''
        if 0<= v and v<= 50: 
            color = 'Black'
            return color
        if 200<= v and v <= 255 :
            color = 'White'
            return color   #マスク着用
        if (0<=s and s<=25) and (199<=v and v<=255):
            color = 'White'
            return color
        elif 90 <= v and 199 <= v :
            color = 'Gray'
            return color
        if (110<=h and h<=130) and (120<=s and s<=160):
            color = 'Brown' #黒人
            return color
         #elif 110<=h and h<=130: color = 'Red'
        if 100<=h and h<=110:
            color = 'Orange' #Asia
            return color
        if (100<=h and h<= 120) and (120<=v and v<=150):
            color = 'skin'
            return color
        if 75<=h and h<=99:
            color = 'Yellow'#Asia
            return color
        elif 50<=h and h<=74: color = 'Green'
        elif 0<=h and h<=20: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 120<=h and h<=135: color = 'Pink' #白人?
        return color
        '''
    def main(self, _):
        response = SetStrResponse()

        #ここを下側に向けたい
        #ズボンの色を検出したいから
        #hipとankleが見えればおけ
        #self.head_pub.publish(-20.0)
        #rospy.sleep(2.5)

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
        
        leg_length = int(rankle_x - rhip_x)
        leg_axis_x = int(rhip_x)
        leg_axis_y = int(rhip_y)
        width      = int(rhip_x - lhip_x)
        if width < 0:
            width = -1*width
        
        print("-----------------------")
        print("leg_length", leg_length)
        print("leg_axis_y",leg_axis_y)
        print("leg_axis_x",leg_axis_x)
        print("width", width)


        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']
        for i in range(leg_length):
            x = i + leg_axis_x
            #if x>479: continue
            for j in range(width):
                y = j + leg_axis_y
                #if y>639: continue
                color = self.judgeColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print(color_map)
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]

        print(response.result)
        return response

if __name__ == '__main__':
    rospy.init_node('detect_cloth_color')
    detect_pants_color = DetectPantsColor()
    print("start")
    rospy.spin()
