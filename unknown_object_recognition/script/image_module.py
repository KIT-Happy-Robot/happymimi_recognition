#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import math
import numpy as np
import pathlib 
from pathlib import Path
import base64
from PIL import Image as PILImage
import cv2
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge


class ImageModule():
    def __init__(self):
        print("Initializing Image Object") 
        self.bridge = CvBridge()
        self.parent_dir = Path(__file__).parent.resolve()
        self.image_dir = self.parent_dir.parent/"image/" # ,,/unknown_object_recognition/image
        self.meter = rosparam.get_param('/camera/realsense2_camera/max_depth_limit')
    def rosInit(self, head=True, head_depth=False, arm==False, arm_depth=False, depth_musk=True):
        rospy.loginfo(f"\nImageHub: Initializing ROS: head_depth:{head_depth}, arm:{arm}, arm_depth:{arm_depth}")
        if head:
            if not depth_musk: rospy.Subscriber('/camera/color/image_raw', Image, self.headColorCB, queue_size=1)
            else: rospy.Subscriber('camera/color/depth_mask',Image,queue_size=10, self.headDepthMuskColorCB)
                #self.pub = rospy.Publisher('camera/color/depth_mask',Image,queue_size=10)
        if head_depth: rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.armColorCB, queue_size=1)###
        if arm: rospy.Subscriber('/camera/color/image_raw_arm', Image, self.armColorCB, queue_size=1)
        if arm_depth: rospy.Subscriber('/camera/depth/image_rect_raw_arm', Image, self.armColorCB, queue_size=1)###
        #rospy.Subscriber('/camera/color/image_raw_arm', Image, self.CB, queue_size=1)
        #rospy.Subscriber('/yolo_result', Image, self.yoloCB)
    def headColorCB(self, msg): self.head_color_image = msg
        # sensor_msgs/ImageメッセージをOpenCVのcv::Mat形式に変換
        # データ型: uint8 (8ビットの符号なし整数), チャンネル数: 3 (RGB各チャンネル), 画像サイズ: (画像の高さ, 画像の幅)
    def headDepthMuskColor(self, msg): self.head_depth_musk_color_image = msg
    def headDepthCB(self, msg): self.head_depth_image = msg
    def armColorCB(self, msg): self.arm_color_image = msg
    def armDepthCB(self, msg): self.arm_depth_image = msg
    def getHeadCvImage(self): return self.bridge.imgmsg_to_cv2(self.head_color_image) #,desired_encoding="rgb8")
    def getArmCvImage(self): return self.bridge.imgmsg_to_cv2(self.arm_color_image)
    def getHeadJpegImage(self): # CV -> Jpeg
        jpeg_data = cv2.imencode(".jpg", self.getHeadCvImage())[1].tobytes() # (成功フラグ, エンコードデータ)のタプル2要素目をバイト列に変換
        with open("head_image.jpg", "wb") as f:
            f.write(jpeg_data)
        return jpeg_data
    def getHeadPngImage(self): # CV -> PNG
        png_data = cv2.imencode(".png", self.getHeadCvImage())[1].tobytes()
        with open("head_image.png", "wb") as f:
            f.write(png_data)
        return png_data
    def getImageFormat(self, image):
        print("\nGetting image format...")
        if isinstance(image, Image):
            print("Input data is a ROS Image message"); return "ros_mage"
        if isinstance(image, np.ndarray):
            print("Input data is an OpenCV cv::Mat"); return "cv"
        try:
            image.tobytes()[:4] == b"\xff\xd8\xff\xe0"
            print("Input data is a JPEG image"); return "jpg"
        except: pass
        try:
            image.tobytes()[:8] == b"\x89\x50\x4e\x47\x0d\x0a\x1a\x0a"
            print("Input data is a PNG image"); return "png"
        except: pass
        # その他の場合
        print("Input data is not a recognized image format")
        return "unknown"
    def convertRosCv(self, ros_image): return self.bridge.imgmsg_to_cv2(ros_image)
    def convertJpgCv(self, jpg_image):
        np_arr = np.frombuffer(jpg_image, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return cv_image
    def converPngCv(self, png_image):
        np_arr = np.frombuffer(png_image, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return cv_image
    def convertCvRos(self, cv_image): return self.bridge.cv2_to_imgmsg(cv_image) #encoding="bgr8")
    def convertCvBase64(self, cv_image):
        _, encoded_image = cv2.imencode('.png', cv_image)
        return base64.b64encode(encoded_image).decode('utf-8')
    def converJpgRos(self, jpg_image):
        np_arr = np.frombuffer(jpg_image, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        ros_image = self.convertCvRos(cv_image)
        return ros_image
    def convertPngRos(self, png_image):
        np_arr = np.frombuffer(png_image, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        ros_image = self.convertCvRos(cv_image)
        return ros_image
    def convertRosJpg(self, ros_img):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(ros_img)
            cv2.imwrite("output.jpg", cv_img)
            ret, jpg_image_data = cv2.imencode(self.image_dir/'ros_to_png.jpg', cv_img) # Bool, jpeg data
            rospy.loginfo("Image data stored in jpg_image_data variable")
        except Exception as e: rospy.logerr(e)
        return jpg_image_data
    def convertRosPng(self, ros_img): # save
        cv_img = self.convertRosCv(ros_img)
        _, png_data = cv2.imencode(self.image_dir/'ros_to_png.png', cv_img)
        png_image_data = np.array(png_data).tobytes() #バイト列
        rospy.loginfo("PNG image data stored in variable")
        # Save
        # file_name = os.path.join(self.image_dir, "converted_image.png")
        # with open(file_name, "wb") as file:
        #     file.write(png_image_data)
        # rospy.loginfo("Image saved as output_image.png")
        return png_image_data

    def autoConvert(self, image, target_format):
        pre = self.getImageFormat(image)
        if target_format == "ros_image":
            if pre == "ros_image": return image
            if pre == "cv": return self.convertCvRos(image)
            if pre == "jpg": return self.converJpgRos(image)
            if pre == "png": return self.convertPngRos(image)
        if target_format == "cv":
            if pre == "ros_imaege": return self.convertRosCv(image)
            if pre == "cv": return image
            if pre == "jpg": return self.convertJpgCv(image)
            if pre == "png": return self.converPngCv(image)
        if target_format == "jpg":
            if pre == "ros_imaege": return self.convertRosCv(image)
            if pre == "cv": return image
            if pre == "jpg": return image
            if pre == "png": return self.converPngCv(image)
        if target_format == "png":
            if pre == "ros_imaege": return self.convertRosCv(image)
            if pre == "cv": return image
            if pre == "jpg": return self.convertJpgCv(image)
            if pre == "png": return self.converPngCv(image)
    
    def getMinDepth(self, image): pass ###
