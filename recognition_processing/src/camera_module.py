#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import rospy
import numpy as np
#from happymimi_recognition_msgs.srv import depth_meter,depth_meterResponse

#from ultralytics import YOLO
#model = YOLO('yolov8n-seg.pt')

class Camera_Module(object):
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw',Image, self.camera_func)
        #rospy.Subscriber()
        self.pub_data = rospy.Publisher('/camera/color/pub_data', Image, queue_size=10)
        
        # 四角形の面積の閾値
        self.min_area_threshold = 1500
        
    def is_quadrilateral(self,contour):
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return len(approx) == 4 and cv2.isContourConvex(approx) and cv2.contourArea(approx) > self.min_area_threshold

    def get_top_left_point_and_bottom_right(self,contour):
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
        return (list(approx[0][0]), list(approx[2][0])) #左上,右下  
    
    def camera_func(self,data):
        
        color_data = self.bridge.imgmsg_to_cv2(data,"bgr8")
        # 画像をグレースケールに変換
        gray = cv2.cvtColor(color_data, cv2.COLOR_BGR2GRAY)

        # 画像のエッジ検出（Cannyエッジ検出を使用）
        edges = cv2.Canny(gray, 50, 150)

        # エッジ画像から輪郭抽出
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 輪郭を描画するコピーを作成
        result_frame = color_data.copy()

        # 最大の輪郭を取得
        max_contour = max(contours, key=cv2.contourArea)

        # 最大輪郭を用いて平面を検出
        #for contour in contours:
        if self.is_quadrilateral(max_contour):
            img_draw = cv2.drawContours(result_frame, [max_contour], 0, (0, 255, 0), 2)
            
        try:
            img_data = self.bridge.cv2_to_imgmsg(img_draw, encoding='bgr8')
        except UnboundLocalError:
            img_data = self.bridge.cv2_to_imgmsg(color_data, encoding='bgr8')
         
        try:
            # 左上,右下点の座標を取得して表示
            point = self.get_top_left_point_and_bottom_right(max_contour)
        except IndexError:
            point = [[0,0],[0,0]]
        
        print(point) 
        self.pub_data.publish(img_data)
            

 
if __name__ == '__main__':
    rospy.init_node("camera_module")
    Camera_Module()
    rospy.loginfo("Start node")
    rospy.spin()
    