#!/usr/bin/env python3
#-*- coding: utf-8 -*



import cv2
import rospy
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import mediapipe as mp
import math

# Carry My Luggage用の左右の荷物判定プログラム
# 骨格検知と指差す方向のものを検知する。
class Directon_finder():
    def __init__(self):
        rospy.init_node('DetectLRHands',anonymous=True)
        # 初期変数
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.pub =rospy.Publisher("direction_of_hands",String,queue_size=10)
        self.bridge = CvBridge()
        rospy.Subscriber('camera/color/image_raw',Image,self.img_listener)
        self.r = rospy.Rate(2)

    def img_listener(self,img):
        self.img = self.bridge.imgmsg_to_cv2(img,"bgr8")
        results = self.pose.process(self.img)
        resultsText = ""
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # 直線aの計算 (ID 12と24の座標から) 右手の肩と右胴体
            a_x1 = landmarks[12].x
            a_y1 = landmarks[12].y
            a_x2 = landmarks[24].x
            a_y2 = landmarks[24].y
            
            # 直線bの計算 (ID 12と14の座標から)　右手の肩と右肘
            b_x1 = landmarks[12].x
            b_y1 = landmarks[12].y
            b_x2 = landmarks[14].x
            b_y2 = landmarks[14].y

            # 直線cの計算 (ID 11と23の座標から)　左手の肩と左胴体
            c_x1 = landmarks[11].x
            c_y1 = landmarks[11].y
            c_x2 = landmarks[23].x
            c_y2 = landmarks[23].y

            # 直線dの計算 (ID 11と13の座標から)　左手の肩と左肘
            d_x1 = landmarks[11].x
            d_y1 = landmarks[11].y
            d_x2 = landmarks[13].x
            d_y2 = landmarks[13].y


            # 2つの直線のなす角度を計算
            angle1 = math.acos((a_x2 - a_x1) * (b_x2 - b_x1) + (a_y2 - a_y1) * (b_y2 - b_y1) / (math.sqrt((a_x2 - a_x1)**2 + (a_y2 - a_y1)**2) * math.sqrt((b_x2 - b_x1)**2 + (b_y2 - b_y1)**2)))
            angle1 = math.degrees(angle1)

            # # 2つの直線のなす角度を計算
            angle2 = math.acos((c_x2 - c_x1) * (d_x2 - d_x1) + (c_y2 - c_y1) * (d_y2 - d_y1) / (math.sqrt((c_x2 - c_x1)**2 + (c_y2 - c_y1)**2) * math.sqrt((d_x2 - d_x1)**2 + (d_y2 - d_y1)**2)))
            angle2 = math.degrees(angle2)

            # 角度が30度を超えた場合にプリント
            if angle1 > 30:
                print(f"角度: {angle1:.2f}度 (30度を超えました) 右手")
                resultsText = "Right"
                rospy.loginfo(resultsText)
                self.pub.publish(resultsText)


            if angle2 > 30:
                print(f"角度: {angle2:.2f}度 (30度を超えました) 左手")
                resultsText = "Left"
                rospy.loginfo(resultsText)
                self.pub.publish(resultsText)
            else:
                resultsText = "None"
                rospy.loginfo(resultsText)
                self.pub.publish(resultsText)
            

        

                # 判定した結果を
if __name__ == '__main__':
    try:
        detect = Directon_finder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logdebug("FINISHED DETECTING HANDS.py")
        cv2.destroyAllWindows()