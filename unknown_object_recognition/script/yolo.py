#!/usr/bin/env python3

# Yolo-worldで特定のラベルのリストの情報に従って、
# 未知の物体の位置、物体名などの情報を返すROSサービスサーバーノード
# uor_moduleのYoloクラスやその関数を使って、未知物体のバウンディングボックスを取得やラベル名の取得をする。
# IN: 

import os
import inspect # DEBUG
import yaml
import numpy as np
from pathlib import Path
import cv2
import torch
from ultralytics import YOLOWorld
import rospy
import cv_bridge

from ultralytics_ros.msg import YoloResult
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from happymimi_msgs.srv import StrTrg # for Finding
from happymimi_recognition_msgs import UorYolo, UorYoloResponse
from image_module import ImageModule
import label_tmp


class YoloHub():
    def __init__(self, model_name=None):
        # Classes list
        self.pkg_dir = Path(__file__).parent.resolve().parent
        with open(self.pkg_dir/"config/uor_model.yaml", 'r') as file:
            self.uor_model_config = yaml.safe_load(file)
        with open(self.pkg_dir/"config/object_class.yaml", 'r') as file:
            self.object_class_list = yaml.safe_load(file)
        self.default_classes = self.object_class_list["default"]
        self.tidyup_classes = self.object_class_list["tidyup"]
        self.lt = label_tmp ###
        self.lt_tu = self.tl.tidyup_classes
        # Set Config
        self.device = self.getDevice()
        self.setClasses()
        rospy.loginfo("\nInitializing YOLO Object")
        if model_name is None: model_name = self.uor_model_config["yolo"]["model"]
        rospy.loginfo(f"Loading model: %s" % model_name)
        # load
        self.model = YOLOWorld(model_name)
        # ROS
        self.bridge = cv_bridge.CvBridge()
        self.IM = ImageModule; self.IM.rosInit()
        rospy.Subscriber(self.uor_model_config["yolo"]["input_topic"], 
                        Image, self.imageCB, queue_size=1, buff_size=2**24)
        self.result_image_pub = rospy.Publisher(self.uor_model_config["yolo"]["input_topic"], 
                                                Image, 
                                                queue_time=1) # Yolo画像パブリッシュ
        self.results_pub = rospy.Publisher("/uor/yolo_result", YoloResult, queue_time=1)
        
        # data type
        self.pseudo_results = []
        self.element_type = {'class_id': 0,'bbox': [], 'conf': 0.0, 'class_prob': 0.0}
        self.class_id_to_name = {} # id:"class_name",,,

    # ultralytics_rosのtrackerノードから持ってきただけ
    def imageCB(self, msg):
        YR = YoloResult()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(cv_image, save=True)
        
        # results = self.model.track(###
        #     source=cv_image,
        #     conf=self.conf_thres,
        #     iou=self.iou_thres,
        #     max_det=self.max_det,
        #     classes=self.classes,
        #     tracker=self.tracker,
        #     device=self.device,
        #     verbose=False,
        #     retina_masks=True)
        # if results is not None:
        #     yolo_result_msg = YoloResult()
        #     yolo_result_image_msg = Image()
        #     yolo_result_msg.header = msg.header
        #     yolo_result_image_msg.header = msg.header
        #     yolo_result_msg.detections = self.createDetectionsArray(results)
        #     yolo_result_image_msg = self.create_result_image(results)
        #     if self.use_segmentation:
        #         yolo_result_msg.masks = self.create_segmentation_masks(results)
        #     self.results_pub.publish(yolo_result_msg)
        #     self.result_image_pub.publish(yolo_result_image_msg)

    def createDetectionsArray(self, results):
        detections_msg = Detection2DArray()
        bounding_box = results[0].boxes.xywh
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf
        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = Detection2D()
            detection.bbox.center.x = float(bbox[0])
            detection.bbox.center.y = float(bbox[1])
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)
            hypothesis.score = float(conf)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)
        return detections_msg    
    
    def createResultImage(self, results):
        plotted_image = results[0].plot(
            #conf=self.result_conf,
            line_width=self.result_line_width,
            font_size=self.result_font_size,
            font=self.result_font,
            #labels=self.result_labels,
            #boxes=self.result_boxes,
        )
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        #cv2.imshow("yolov8",plotted_image)
        #cv2.waitKey(1)
        return result_image_msg

    def createSegmentationMasks(self, results):
        masks_msg = []
        for result in results:
            if hasattr(result, "masks") and result.masks is not None:
                for mask_tensor in result.masks:
                    mask_numpy = (
                        np.squeeze(mask_tensor.data.to("cpu").detach().numpy()).astype(
                            np.uint8
                        )
                        * 255
                    )
                    mask_image_msg = self.bridge.cv2_to_imgmsg(
                        mask_numpy, encoding="mono8"
                    )
                    masks_msg.append(mask_image_msg)
        return masks_msg

    def getDevice(self):
        device = "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda":
            if self.uor_model_config["device"] == "gpu": pass
            else: device = "cpu"
        return device
    
    # Yolo-worldの推論実行して生で返す
    def executePredict(self, image, classes):
        self.class_id_list = {id: name for id, name in enumerate(classes, start=0)} # id
        self.model.set_classes(classes)
        results = self.model.predict(image, classes)
        return results
    # DEBUG
    def debugPredict(self):
        results = self.executePredict(self.IM.head_depth_musk_color_image, self.lt_tu) # tidyup:lt_tu or tidyup_classes, default:
        results[0].show()
        print("\nresults[0] :" + results[0])
        print("\n results[0].names:" + results[0].names)
        print("\n display results[0] image by cv2")
        result_image = results[0]
        cv2.imshow(result_image)
        
    def getResultList(self, results): # IN:Yolo results, OUT: label and bbox list
        detections = []
        for result in results.xyxy[0].tolist():  # Loop through detections in the first image
            x1, y1, x2, y2, confidence, class_id = result[:6]
            class_name = self.model.names[int(class_id)]
            detections.append([class_name, x1, y1, x2, y2], confidence)
            print(f"Detected {class_name} with confidence {confidence} at [{x1}, {y1}, {x2}, {y2}]")
        return detections
