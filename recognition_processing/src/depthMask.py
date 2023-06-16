#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import rospy
import numpy as np

#以下でlaunchを起動する必要あり
#roslaunch realsense2_camera rs_camera.launch align_depth:=True

#画像サイズ
WIDTH = 640
HEIGHT = 480

class ImageMask():
    def __init__(self):
        rospy.init_node('depsMaskImage',anonymous=True)
        self.bridge = CvBridge()
        #Realsenseのカラー画像と深度情報取得
        rospy.Subscriber('/camera/color/image_raw',Image,self.img_listener)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depth_listener)
        self.pub = rospy.Publisher('camera/color/depth_mask',Image,queue_size=10)
    
    #カラー画像を処理
    def img_listener(self,data):
        try:
            color_data = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.img = np.copy(color_data)
            self.img = cv2.resize(self.img,(WIDTH,HEIGHT))
        except CvBridgeError as e:
            print("img:",e)

    #深度情報を処理
    def depth_listener(self,data):
        try:
            depth_data = self.bridge.imgmsg_to_cv2(data,"32FC1")
            self.depth = depth_data * 0.001
            self.depth = cv2.resize(self.depth,(WIDTH,HEIGHT))
            rospy.sleep(0.01)
            self.maskImg()
        except CvBridgeError as e:
            print("depth:",e)
        
    #meter以上をマスクして黒にする
    def maskImg(self,meter=2.5):
        try:
            mask1 = np.zeros((self.img.shape[0],self.img.shape[1],3), np.uint8)
            mask2 = np.zeros((self.img.shape[0],self.img.shape[1],3), np.uint8)
            mask1[self.depth < meter] = (255,255,255)
            mask2[self.depth != 0] = (255,255,255)
            mask = cv2.bitwise_and(mask1,mask2)
            img_AND = cv2.bitwise_and(self.img,mask)


            img_pub = self.bridge.cv2_to_imgmsg(img_AND,encoding="bgr8")
            self.pub.publish(img_pub)

        except cv2.error as e:
            print(e)
        except:
            print("Error")



if __name__ == '__main__':
    try:
        ImageMask()
        rospy.spin()
    except rospy.ROSInitException:
        print('Shutting down')
        cv2.destroyAllWindows()
