#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
#from happymimi_recognition_msgs.srv import MultipleLocalize,MultipleLocalizeResponse
from geometry_msgs.msg import Point
from ros1_yolov8.srv import LeftRight2xyz,LeftRight2xyzResponse

class DetectPaperBag():
    def __init__(self):
        rospy.init_node('DetectPaperBag',anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO("/home/ayu/catkin_ws/src/ros1_yolov8/src/paper_bag.pt")
        #self.model = YOLO("yolov8n.pt")
        #rospy.Subscriber('/yolo_image',Image,self.img_listener)  
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,self.depth_listener)
        rospy.Subscriber('/camera/color/image_raw',Image,self.img_listener)
        #self.pub = rospy.Publisher("Coordinate_PaperBag",MultipleLocalize,queue_size=10)
        srv = rospy.Service('Coordinate_PaperBag',LeftRight2xyz,self.Coordinate_srv)
        self.r = rospy.Rate(2)


    def clear_val(self):
        self.size_x = []
        self.size_y = []
        self.center_x = []
        self.center_y = []
        self.x1 = []
        self.y1 = []
        self.x2 = []
        self.y2 = []


    def img_listener(self,img):
        self.img = self.bridge.imgmsg_to_cv2(img,"bgr8")
        self.clear_val()
        results = self.modelre(source=self.img,conf=0.7)
        boxes = results[0].boxes
        for box in boxes:
            x,y,w,h = [int(i) for i in box.xywh[0]]
            x1,y1,x2,y2 = [int(i) for i in box.xyxy[0]]
            self.center_x.append(x)
            self.center_y.append(y)
            self.size_x.append(w)
            self.size_y.append(h)
            self.x1.append(x1)
            self.y1.append(y1)
            self.x2.append(x2)
            self.y2.append(y2)
        self.Coordinate_Paperbag()
    

    def depth_listener(self,data):
        try:
            depth_data = self.bridge.imgmsg_to_cv2(data,"32FC1")
            self.depth = depth_data * 0.001     #単位を[m]にする
        except CvBridgeError as e:
            print("depth_listener:",e)


    def Coordinate_Paperbag(self):
        self.put_x1 = []
        self.put_y = []
        self.put_depth = []
        for i in range(len(self.center_x)):
            self.put_x1.append(self.center_x[i] - (self.center_x[i] - self.x1[i]) / 4) #持ち手の左
            self.put_y.append(self.center_y[i] - (self.center_y[i] - self.y1[i]) / 1.4) #持ち手の高さ
            self.put_depth.append(self.depth[self.center_y[i]][self.center_x[i]])
        
        try:
            self.put_x1,self.put_y,self.put_depth = zip(*sorted(zip(self.put_x1,self.put_y,self.put_depth)))
            print("x:{}\ny:{}\nz:{}".format(self.put_x1,self.put_y,self.put_depth))
        except:
            self.put_x1 = [-1,-1]
            self.put_y = [-1,-1]
            self.put_depth = [-1,-1]
        self.r.sleep()



    def Coordinate_srv(self,LeftRight):
        if LeftRight == "Left":
            return LeftRight2xyzResponse(self.put_x1[0],self.put_y[0],self.put_depth[0])
        else:
            return LeftRight2xyzResponse(self.put_x1[-1],self.put_y[-1],self.put_depth[-1])


if __name__ == '__main__':
    try:
        DetectPaperBag()  
        rospy.spin()   
    except rospy.ROSInitException:
        print('Shutting down')
        cv2.destroyAllWindows()
