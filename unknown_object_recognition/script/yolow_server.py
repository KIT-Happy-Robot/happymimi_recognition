#!/usr/bin/env python3
#-*- coding: utf-8 -*

import os
import cv2
import sys
import numpy    
from pathlib import Path
from ultralytics import YOLOWorld
import rospy
import roslib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from happymimi_msgs.srv import SetStr
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest
import yaml

# Initialize the model with pre-trained weights
#model = YOLOWorld('../model/yolov8m-world.pt')

# Set what you'd like to find in your image
#model.set_classes(["mouse pad"])

yaml_path = roslib.packages.get_pkg_dir("unknown_object_recognition") + '/config/'
model_path = roslib.packages.get_pkg_dir("unknown_object_recognition") + '/model/'
sys.path.insert(0,model_path)
param_yaml_name = "uor_server_config.yaml"
point_data_yaml_name = "uor_object_location.yaml"

recognition_path = roslib.packages.get_pkg_dir("recognition_processing") + '/src/'
sys.path.insert(1,recognition_path)
from recognition_tools import CallDetector

with open(yaml_path+param_yaml_name, 'r') as file:
    yaml_data = yaml.safe_load(file)

class YoloWorld_Server():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.yolo_topic)
        #rospy.Subscriber('/camera/color/depth_mask', Image, self.yolo_topic)
        rospy.Service('/uor/yolo_server', SetStr,self.yolo_main)
        rospy.set_param('/uor/config', yaml_data)
                
        self.config = rospy.get_param('/uor/config', {})
        model_name = ''.join(self.config["model"])
        self.conf = self.config["confidence"]
        self.model_class = self.config["item"]
        #with open(os.path.join(Path(__file__).parent.resolve().parent), "config/object_class_list.yaml", 'r') as file:
        #    self.object_class_list = yaml.safe_load(file)
        #self.tu_items = self.object_class_list["yumeko_tu"]##
        
        self.model = YOLOWorld(model=model_name)
        self.model.set_classes(self.model_class) # self.model_class)
        
        self.jpeg_data = None
        self.data = None
        self.point_data = []
        self.point_data_dict = []
        
    def yolo_topic(self, res):
        self.data = self.bridge.imgmsg_to_cv2(res,"bgr8")
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 例として圧縮率90で設定
        result, self.jpeg_data = cv2.imencode('.jpeg', self.data ,encode_param)
    
    def to_list(self, data):
        if isinstance(data, list):  # データがリスト型かどうかを確認
            return data
        else:
            return [data]  # データを単一要素のリストに変換して返す
    
    def param_update(self):
        self.config = rospy.get_param('/uor/config', {})
        self.model_name = ''.join(self.config["model"])
        self.conf = self.config["confidence"]
        
        self.model_class = self.config["item"]
        self.model_class = self.to_list(self.model_class)
        
        self.model = YOLOWorld(model=self.model_name)
        self.model.set_classes(self.model_class)
    
    def LocalizeObjectWorld(self,x_point, y_point):
        Detector = CallDetector()
        Detector.detectorService(x_point, y_point)
        return Detector.object_centroid
    
    def yolo_main(self, _):
        
        centroid = Point()
        centroid.x = numpy.nan
        centroid.y = numpy.nan
        centroid.z = numpy.nan
        
        # JPEG形式のバイナリデータをファイルに保存する場合
        with open("output.jpg", "wb") as f:
            f.write(self.jpeg_data.tobytes())
        
        self.param_update()
        ##debug用
        image = cv2.imread("output.jpg")
        ##
        results = self.model.predict('output.jpg',conf=self.conf[0])
        if results[0].boxes:
            for j in range(len(results[0].boxes)):
                x_point,y_point,w,h =[int(i) for i in results[0].boxes.xywh[j]]
                
                print("x_point, y_point:",x_point, y_point)
                centroid = self.LocalizeObjectWorld(x_point=x_point, y_point=y_point)
                self.point_data.append([centroid.x, centroid.y, centroid.z])
            
                id = int(results[0].boxes[j].cls)
                for i,coord in enumerate(self.point_data):
                    key = results[0].names[id]
                    self.point_data_dict.setdefault(key,coord)
        
        else:
            self.point_data_dict = {}
        
        # Show results
        results[0].show()
        
        with open(yaml_path+point_data_yaml_name, 'w') as file:
            yaml.dump(self.point_data_dict, file)
        
        return "True"
    
    def load_yaml_to(self,file_path):
        point = Point()
        with open(file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            if yaml_data:
                for key, value in yaml_data.items():
                    if isinstance(value, list) and len(value) == 3:
                        point.x = value[0]
                        point.y = value[1]
                        point.z = value[2]
                        return point
                    else:
                        rospy.logwarn("Invalid data format for key %s", key)
                        point.x = numpy.nan
                        point.y = numpy.nan
                        point.z = numpy.nan
                        return point
    
if __name__ == '__main__':
    rospy.init_node("yolo_topic")
    YoloWorld_Server()
    rospy.loginfo("Start node")
    rospy.spin()