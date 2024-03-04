#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import rospy
import tensorflow as tf
import tensorflow_hub as hub
from happymimi_msgs.msg import SimpleBool
from ultralytics_ros.msg import YoloResult
from geometry_msgs.msg import Point
from recognition_tools_v8 import RecognitionToolsV8
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest
from happymimi_msgs.srv import SimpleTrg

KEYPOINT_THRESHOLD = 0.2
model = hub.load('https://tfhub.dev/google/movenet/multipose/lightning/1')


class Raise_Hand(object):
    def __init__(self):
        rospy.Subscriber('/yolo_image', Image,self.image_callback)
        rospy.Service("/raise_your_hand",SimpleTrg, self.main)
        self.detect_depth = rospy.ServiceProxy('/detect/depth', PositionEstimator)
        self.object_centroid = Point()        
        self.bridge = CvBridge()
        self.movenet = model.signatures['serving_default']        
        self.center_list = []
        self.result_list = []
    
    def image_callback(self,res):
        self.frame = res
    
    def run_inference(self, image):
            # 画像の前処理
        input_image = cv2.resize(image, dsize=(256, 256))
        input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
        input_image = np.expand_dims(input_image, 0)
        input_image = tf.cast(input_image, dtype=tf.int32)

        # 推論実行・結果取得
        outputs = self.movenet(input_image)
        keypoints = np.squeeze(outputs['output_0'].numpy())
        
        image_height, image_width = image.shape[:2]
        keypoints_list, scores_list, bbox_list = [], [], []

        # 検出した人物ごとにキーポイントのフォーマット処理
        for kp in keypoints:
            keypoints = []
            scores = []
            for index in range(17):
                kp_x = int(image_width * kp[index*3+1])
                kp_y = int(image_height * kp[index*3+0])
                score = kp[index*3+2]
                keypoints.append([kp_x, kp_y])
                scores.append(score)
            bbox_ymin = int(image_height * kp[51])
            bbox_xmin = int(image_width * kp[52])
            bbox_ymax = int(image_height * kp[53])
            bbox_xmax = int(image_width * kp[54])
            bbox_score = kp[55]

            keypoints_list.append(keypoints)
            scores_list.append(scores)
            bbox_list.append([bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax, bbox_score])

        return keypoints_list, scores_list, bbox_list
    
    # 挙手を判定する関数
    def is_raise_hand(self,keypoints_list, scores_list, bbox_list):
        for i, (keypoints, scores, bbox) in enumerate(zip(keypoints_list, scores_list, bbox_list)):
            if bbox[4] < 0.2:
                continue
            if keypoints[9][1] > keypoints[5][1] and keypoints[10][1] > keypoints[6][1]: # 鼻より上に手首がないとき
                return False
            self.center_list.append(bbox)
        return True

    def main(self, _):
        rospy.wait_for_service('/detect/depth')
        position_estimator_req = PositionEstimatorRequest()
        color_data = self.bridge.imgmsg_to_cv2(self.frame,"bgr8")
        
        # 推論実行
        keypoints_list, scores_list, bbox_list = self.run_inference(color_data)
        self.is_raise_hand(keypoints_list, scores_list, bbox_list)
        for center in range(len(self.center_list)):
            position_estimator_req.center_x = int(self.center_list[center][2] - self.center_list[center][0])
            position_estimator_req.center_y = int(self.center_list[center][3] - self.center_list[center][1])
            result = self.detect_depth(position_estimator_req)
            self.result_list.append(result.point)
            print(self.result_list)
                        
        return True

if __name__ == '__main__':
    rospy.init_node("raise_your_hands")
    Raise_Hand()
    rospy.loginfo("Start node")
    rospy.spin()
    