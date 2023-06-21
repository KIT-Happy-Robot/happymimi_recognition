#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import rospy
import numpy as np
from mimi_motion_detection.srv import depth_meter,depth_meterResponse


#以下でlaunchを起動する必要あり
#roslaunch realsense2_camera rs_camera.launch align_depth:=True

#画像サイズ
WIDTH = 640
HEIGHT = 480

class ImageMask():
    def __init__(self):
        rospy.init_node('depsMaskImage',anonymous=True)
        self.bridge = CvBridge()
        self.meter = 2.5
        #Realsenseのカラー画像と深度情報取得
        rospy.Subscriber('/camera/color/image_raw',Image,self.img_listener)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depth_listener)
        self.pub = rospy.Publisher('camera/color/depth_mask',Image,queue_size=10)
        self.server = rospy.Service("depth_mask",depth_meter,self.depth_reception)
        
    def depth_reception(self,request):
        self.meter = float(str(request).split(":")[1])
        return depth_meterResponse("depth_mask is {}[m]".format(self.meter))

    #カラー画像を処理
    def img_listener(self,data):
        try:
            color_data = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.img = np.copy(color_data)
            self.img = cv2.resize(self.img,(WIDTH,HEIGHT))
        except CvBridgeError as e:
            print("img_listener:",e)

    #深度情報を処理
    def depth_listener(self,data):
        try:
            depth_data = self.bridge.imgmsg_to_cv2(data,"32FC1")
            self.depth = depth_data * 0.001     #単位を[m]にする
            self.depth = cv2.resize(self.depth,(WIDTH,HEIGHT))
            rospy.sleep(0.01)
            self.maskImg()
        except CvBridgeError as e:
            print("depth_listener:",e)
        
    #深度を用いた画像のマスク
    def maskImg(self):
        print("pub_{}[m]".format(self.meter))
        ksize = 5   #カーネルサイズ(中央値フィルタで使用(1より大きい奇数で運用))
        try:
            #マスクのためのリストを初期化
            mask1 = np.zeros((self.img.shape[0],self.img.shape[1],3), np.uint8)
            mask2 = np.zeros((self.img.shape[0],self.img.shape[1],3), np.uint8)

            mask1[self.depth < self.meter] = (255,255,255)   #meter[m]より近い点を白
            mask2[self.depth >= 0.5] = (255,255,255)    #0.5[m]以上の距離は白
            mask = cv2.bitwise_and(mask1,mask2)     #meter > self.depth >= 0.5の距離を白
            mask = cv2.medianBlur(mask,ksize)    #中央値フィルタによる小さいノイズ処理
            img_AND = cv2.bitwise_and(self.img,mask)    #カラー画像のマスク
            

            #画像をtopicとして流す
            img_pub = self.bridge.cv2_to_imgmsg(img_AND,encoding="bgr8")
            self.pub.publish(img_pub)

        except cv2.error as e:
            print(e)



if __name__ == '__main__':
    try:
        ImageMask()
        print("throw topic")
        rospy.spin()
    except rospy.ROSInitException:
        print('Shutting down')
        cv2.destroyAllWindows()
