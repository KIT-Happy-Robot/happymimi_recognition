#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import rospy
from happymimi_msgs.msg import SimpleBool
from geometry_msgs.msg import Point
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest
from happymimi_msgs.srv import SimpleTrg


class Raise_Hand(object):
    def __init__(self):
        rospy.Subscriber('/camera/color/image_raw', Image,self.image_callback)
        rospy.Service("/raise_your_hand",SimpleTrg,self.main)
        rospy.Service("raise_your_hands_zero_shot",Image,self.zero_shot_main)
        # self.detect_depth = rospy.ServiceProxy('/detect/depth', PositionEstimator)
        self.object_centroid = Point()        
        self.bridge = CvBridge()

        # mediapipeの準備
        self.mp_hands = mp.solutions.hands
        self.mp_pose = mp.solutions.pose
        #mediapipeの検出モデルを読み込む
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
    
    def image_callback(self,res):
        self.frame = res
        
    def zero_shot_main(self,req):
        print("yobareta")
        color_data = self.bridge.imgmsg_to_cv2(req,"bgr8")
        
        # 推論実行
        # results_hands = self.hands.process(color_data)
        results_pose = self.pose.process(color_data)
        print(results_pose)


        if results_pose.pose_landmarks:
            landmarks = results_pose.pose_landmarks.landmark
            right_shoulder = [landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            right_wrist = [landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

            left_shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            left_wrist = [landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].y]

            # 肩と手首の座標を使用して角度を計算
            right_angle = np.arctan2(right_wrist[1] - right_shoulder[1], right_wrist[0] - right_shoulder[0]) * 180 / np.pi
            left_angle = np.arctan2(left_wrist[1] - left_shoulder[1], left_wrist[0] - left_shoulder[0]) * 180 / np.pi

            #　挙手判定
            if right_angle < 30:
                cv2.putText(color_data, "Right arm is raised", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                rospy.loginfo("Right arm is raised | detect Raise_Hand")
                return True
            elif left_angle < 30:
                cv2.putText(color_data, "Left arm is raised", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                rospy.loginfo("Left arm is raised | detect Raise_Hand")
                return True
            else:
                return False


    def main(self, _):
        print("yobareta")
        # rospy.wait_for_service('/detect/depth')
        color_data = self.bridge.imgmsg_to_cv2(self.frame,"bgr8")
        
        # 推論実行
        # results_hands = self.hands.process(color_data)
        results_pose = self.pose.process(color_data)
        print(results_pose)


        if results_pose.pose_landmarks:
            landmarks = results_pose.pose_landmarks.landmark
            right_shoulder = [landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            right_wrist = [landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

            left_shoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            left_wrist = [landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].y]

            # 肩と手首の座標を使用して角度を計算
            right_angle = np.arctan2(right_wrist[1] - right_shoulder[1], right_wrist[0] - right_shoulder[0]) * 180 / np.pi
            left_angle = np.arctan2(left_wrist[1] - left_shoulder[1], left_wrist[0] - left_shoulder[0]) * 180 / np.pi

            #　挙手判定
            if right_angle < 30:
                cv2.putText(color_data, "Right arm is raised", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                rospy.loginfo("Right arm is raised | detect Raise_Hand")
                return True
            elif left_angle < 30:
                cv2.putText(color_data, "Left arm is raised", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                rospy.loginfo("Left arm is raised | detect Raise_Hand")
                return True
            else:
                return False
        
                        
if __name__ == '__main__':  
    rospy.init_node("raise_your_hands")
    Raise_Hand()
    rospy.loginfo("Start node")
    rospy.spin()
    