#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
from subprocess import PIPE
import subprocess
import math
import numpy as np
import pathlib 
from pathlib import Path
# Process API --------------------------------
import cv2
import torch
import open3d as o3d
import matplotlib.pyplot as plt 
# Remote API --------------------------------
import requests
from transformers import pipeline
import clip
from PIL import Image as PILImage
from transformers import CLIPProcessor, CLIPModel ###
#----------------------------------------------------------------
from sklearn.cluster import DBSCAN ###
# Local API ----------------------------------------------------
from transformers import BlipProcessor, BlipForConditionalGeneration #Instrtion: https://github.com/salesforce/BLIP
from ultralytics import YOLOWorld
# ROS --------------------------------
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
# ROS Custum Messages --------------------------------
from happymimi_msgs.srv import SetStr, SetStrResponse

password = (os.environ["SUDO_KEY"] + "\n").encode()
pkg_dir = Path(__file__).parent.resolve().parent
image_dir = pkg_dir/"image/"
with open(pkg_dir/"config/uor_model.yaml", 'r') as file:
    uor_model_config = yaml.safe_load(file)
def getDevice():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cuda":
        if uor_model_config["device"] == "gpu": pass
        else: device = "cpu"
    return device
device = getDevice()

class ImageHub():
    def __init__(self):
        print("Initializing Image Object") 
        self.bridge = CvBridge()
        self.parent_dir = Path(__file__).parent.resolve()
        self.image_dir = self.parent_dir.parent/"image" # ,,/unknown_object_recognition/image
    def rosInit(self, subs=None):
        rospy.Subscriber('/camera/color/image_raw', Image, self.setHeadColorImageCB, queue_size=1)
        #rospy.Subscriber('/camera/color/image_raw_head', Image, self.armColorImageCB, queue_size=1)###
        #rospy.Subscriber('/camera/color/image_raw_arm', Image, self.CB, queue_size=1)
        #rospy.Subscriber('/yolo_result', Image, self.yoloCB)
    def setHeadColorImageCB(self, msg): self.head_ros_image = msg
    #def setArmColorImageCB(self, msg): self.arm_image = msg
    # sensor_msgs/ImageメッセージをOpenCVのcv::Mat形式に変換
    # データ型: uint8 (8ビットの符号なし整数), チャンネル数: 3 (RGB各チャンネル), 画像サイズ: (画像の高さ, 画像の幅)
    def getHeadCvImage(self): return self.bridge.imgmsg_to_cv2(self.head_ros_image) #,desired_encoding="rgb8")
    #def getArmCvImage(self): return self.bridge.imgmsg_to_cv2(self.arm_image)
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
        _, encoded_image = cv2.imencode('.png', cv)
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
            ret, jpg_image_data = cv2.imencode(image_dir/'ros_to_png.jpg', cv_img) # Bool, jpeg data
            rospy.loginfo("Image data stored in jpg_image_data variable")
        except Exception as e: rospy.logerr(e)
        return jpg_image_data
    def convertRosPng(self, ros_img): # save
        cv_img = self.convertRosCv(ros_img)
        _, png_data = cv2.imencode(image_dir/'ros_to_png.png', cv_img)
        png_image_data = np.array(png_data).tobytes() #バイト列
        rospy.loginfo("PNG image data stored in variable")
        # Save
        # file_name = os.path.join(image_dir, "converted_image.png")
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

class PointCloudHub():
    def __init__(self):
        print("Initializing PointCloudHub object...")
        self.point =[]
        self.points = []
        self.pc2 = []

    def rosInit(self):
        rospy.Subscriber('/camera/depth/points', Image, self.setPointsCB, queue_size=1)
        #rospy.Subscriber('/camera/depth/color/points', Image, self.setDepthImageCB, queue_size=1) ###
        # /camera/depth/image_rect_raw 生の深度値
        
    def setPointsCB(self, msg):
        #pcl = pointcloud2_to_xyz_array(pcl_msg) ###
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(msg.xyz)
        #o3d.visualization.draw_geometries([self.o3d_pcd]) # ポイントクラウドの可視化など
    def getO3dPoints(self): return self.o3d_pcd
    def setDepthImageCB(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(e)
    def getWidthSlice(self, area): ###
        height, width = self.depth_image.shape[:2] # # 画像の幅と高さを取得
        left = int(width / 4) # 左右方向の範囲を定義（画面横幅の1/4）
        right = int(width * 3 / 4)
        depth_slice = self.depth_image[:, left:right] # 指定された範囲の深度データを抽出
        depth_slice_valid = depth_slice[depth_slice > 0] # 深度が0（無効値）のピクセルを除外
    def getHeightSlice(self, area): pass ###
    def getSlice(self, area): pass ###
    def getNearestDistance(self, di):
        if len(id) > 0:
            min_depth = np.min(id) # 最も近い深度を取得
            rospy.loginfo("\n最も浅い深度: {:.2f} メートル".format(min_depth))
        else:
            rospy.loginfo("\ngetNearestDistance: 指定された範囲内に深度データがありません")
        return min_depth # + 0.05
    # Open3d ------------------------
    # downsample
    # 
    # Extraction
    def getPlanePoints(self, option=None, distance_threshold=0.01, ransac_n=3, num_iterations=1000): # 
        # distance_threshold：インライアとするしきい値, ransac_n：平面を推定する最小の点数, num_iterations：試行回数
        plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold, 
                                                            ransac_n, 
                                                            num_iterations)
        [a, b, c, d] = plane_model # each 3d point
        # [a, b, c, d] = plane_model.np.tolist()
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        # DEBUG
        inlier_cloud = self.o3d_pcd.select_by_index(inliers) # インライアの点を抽出して色を付け
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        if option: return self.o3d_pcd.select_by_index(inliers, invert=False)
        if not option: return self.o3d_pcd.select_by_index(inliers, invert=True)
        if option is None: return self.o3d_pcd.select_by_index(inliers, invert=False)
    def getPointOnPlane(self): # 平面上の点群を抽出
        plane_pcd = self.getPlanePoints()
        return 
    
    def extractPointsOnTable(self, ):
    def extractTableObjects(self, ):###
        
        return centroid_num, centroid_coords, a

# 
class Prompt():
    def __init__(self):
        #lists = rospy.get_param('objects_list', [])
        self.loadLists()
        
    def loadLists(self):
        # Tidyup用のオブジェクトリスト
        # with open(pkg_dir/"config/object_list.yaml", 'r') as file:
        #     self.object_dict = yaml.safe_load(file)
        with open(pkg_dir/"config/item_category.yaml", 'r') as file:
            self.object_dict = yaml.safe_load(file)
        # self.tidyup_object_dict = rospy.get_param('item_category')
        self.tidyup_category_list = list(self.tidyup_object_dict.keys()) # (# dict_keys(['',,,]))
        self.tidyup_item_list = [value for values in self.tidyup_object_dict.values() for value in values]

    def multiplClassClasificationPrompt(self):
        self.classify_center_tidyup_object_name = (
            f"Guess the name of the object in the center of this image in object list: {self.tidyup_object_list}")
        self.classify_tidyup_object_category = (
            f"Guess the name of the object category in this image in object list: {self.tidyup_object_list}")

class YoloHub():
    def __init__(self, model_name==None):
        print("\nInitializing YOLO Object")
        if model_name is None: model_name = uor_model_config["yolo"]["model"]
        print(f"Loading model: %s" % model_name)
        self.model = YOLOWorld(model_name)
        
        # 物体検出結果からラベル名のリストを取得
        #detected_labels = [obj.label for obj in yolo_result.objects]
    
    def setClasses(self, classes==None): self.model.set_classes(classes)
    def getResultList(self, results): # IN:Yolo results, OUT:
        detections = []
        for result in results.xyxy[0].tolist():  # Loop through detections in the first image
            x1, y1, x2, y2, confidence, class_id = result[:6]
            class_name = self.model.names[int(class_id)]
            detections.append([class_name, x1, y1, x2, y2], confidence)
            print(f"Detected {class_name} with confidence {confidence} at [{x1}, {y1}, {x2}, {y2}]")
        return detections
    def detectObject(self, classes, image):
        self.model.set_classes(classes)
        results = self.model.predict(image, save=True)
        # Show results
        results[0].show()
        output_img = results[0].plot()
        output_msg = self.bridge.cv2_to_imgmsg(results[0], encoding="bgr8")
        #return output_img
        results_lists = self.getResultList(results)
        return results
        

# Remote ----------------------------------------------------------------
def checkWifi():
    proc = subprocess.run(["sudo","-S","wpa_cli", "status"],stdout = subprocess.PIPE, stderr = subprocess.PIPE, input=password)
    data = proc.stdout.decode("utf8").split()
    if data[5] == "ssid=KIT-WLAP2":
        server = "http://wwwproxy.kanazawa-it.ac.jp:8080"
        servers = "https://wwwproxy.kanazawa-it.ac.jp:8080"
        os.environ["http_proxy"] = server
        os.environ["https_proxy"] = servers
    else:
        server = ""
        os.environ["http_proxy"] = server
        os.environ["https_proxy"] = server

class ClipHub():
    def __init__(self):
        print("Initializing CLIP")
        checkWifi()
        # DIVECE setting
        self.device = device
    #モデルの読み込み ###  
    def loadModel(self, task="clasification"):
        if task=="clasification":
            self.model, self.preprocess = clip.load(uor_model_config["clip"]["model"], device=self.device)
            print("ClipHub: Complete loading model")
            # memo
            #image = preprocess(Image.open("test.png")).unsqueeze(0).to(device)
            #text = clip.tokenize(["a human", "a dog", "a cat"]).to(device)
        #if task=="description": ###
        #captioner = pipeline("image-to-text",model="Salesforce/blip-image-captioning-base")
        #processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")
    
    def objectDetection(self): pass # Out:Bool
    def objectNameClasifiction(self, lavels, image): # IN:text list, cv image | OUT:max prop
        # clip
        # image_features = self.model.encode_image(image)
        # text_features = self.model.encode_text(text_list)
        #logits_per_image, logits_per_text = self.model(image, labels)# 推論
        # CLIP
        inputs = processor(text=labels, images=image, return_tensors='pt', padding=True)
        outputs = model(**inputs)
        logits_per_image = outputs.logits_per_image
        logits_per_text = outputs.logits_per_text
        probs = logits_per_image.softmax(dim=1); print("\nClipHub: probs: "+probs)
        top_class= probs.argmax(-1).item(); print("\nClipHub: top class: "+ top_class)
        return 
    
    def objectColorClasification(self): pass
    def objectCategoryClasification(self): pass
    def binaryClasification(self): pass


    
    def extract_gender(self):
        #試験的に性別を判断する
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_gender = processor(text=self.label_gender, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_gender = model(**inputs_gender)
        logits_per_image = outputs_gender.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_gender[predicted_class_idx])
        print("score:", probs)
        
        return self.label_gender[predicted_class_idx]


import argparse
import base64
from settings.setting import API_KEY

class Gpt4vHub():
    def __init__():
        print("Initializing GPT4vHub Object...")
    
    def parseArgs():
        parser = argparse.ArgumentParser()
        parser.add_argument('-i', '--image', type=str,)
        parser.add_argument('-p', '--prompt', type=str)
        return parser.parse_args()
    
    def encodeImage(self, image_file):
        if image_file is str():
            if "/" in text:
                with open(image_path, "rb") as image_file:
                    return base64.b64encode(image_file.read()).decode('utf-8')
            else: return None
        return base64.b64encode(image_file.read()).decode('utf-8')
    