#!/usr/bin/env python3
#-*- coding: utf-8 -*

"""席にいる二人の人の内、どちらが手を上げているかを判定するプログラム"""
import mask_person
import cv2
import rospy
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from happymimi_msgs.srv import SetStr,SetStrResponse
from std_msgs.msg import String
import math

class RaiseYourHandsTwoPerson():
    def __init__(self):
        rospy.init_node('DetectRaiseYourHandsTwoPeople',anonymous=True)
        
        rospy.Service("raise_your_hands_which_two_people",SetStr,self.take_picture) #ここに向かってどちらの人間が手を上げているのか判定を行う
        self.raiseHandsSrv = rospy.ServiceProxy("raise_your_hands_zero_shot",Image) #挙手判定に関するサービス
        rospy.Subscriber('camera/color/image_raw',Image,self.img_listener) # リアルタイムに画像を受け取る。
        self.bridge = CvBridge()
        self.r = rospy.Rate(2)
    def img_listener(self,req):
        self.img = self.bridge.imgmsg_to_cv2(req,"bgr8") # 画像を受け取る
        self.img = cv2.resize(self.img,(640,480)) # 画像をリサイズする
        
    def take_picture(self,_):
        self.oneframeImage = self.img
        result = self.detectRaiseHandsTwoPeople()
        return SetStrResponse(result)
    
    def detectRaiseHandsTwoPeople(self):
        people1, people2 = mask_person.mask_person(self.oneframeImage)
        people1_imgmsg = self.bridge.cv2_to_imgmsg(people1,"bgr8")
        people2_imgmsg = self.bridge.cv2_to_imgmsg(people2,"bgr8")
        response_people1 = self.raiseHandsSrv(people1_imgmsg)
        response_people2 = self.raiseHandsSrv(people2_imgmsg)
        while response_people1 == True and response_people2 == True:
            response_people1 = self.raiseHandsSrv(people1_imgmsg)
            response_people2 = self.raiseHandsSrv(people2_imgmsg)
        
        if response_people1:
            return "people1"
        elif response_people2:
            return "people2"
            
        
        
        
        
        
        
        
        
