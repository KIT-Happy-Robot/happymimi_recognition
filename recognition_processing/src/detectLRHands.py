#!/usr/bin/env python3
#-*- coding: utf-8 -*



import cv2
import rospy
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import mediapipe as mp

# Carry My Luggage用の左右の荷物判定プログラム
# 骨格検知と指差す方向のものを検知する。
class DetectLRArrowHands():
    def __init__(self):
        rospy.init_node('DetectLRHands',anonymous=True)
        # 初期変数
        self.mp_hands = mp.solutons.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)

    


        self.bridge = CvBridge()
        rospy.Subscriber('camera/color/image_raw',Image,self.img_listener)
        self.r = rospy.Rate(2)
    
    def img_listener(self,img):
        self.img = self.bridge.imgmsg_to_cv2(img,"bgr8")
        results = self.hands.process(self.img)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                #手の座標を取得
                f_max = 0
                y_max = 0
                x_min = float('inf')
                y_min = float('inf')

                for landmark in hand_landmarks.landmark:
                    x = min(int(landmark.x * self.img.shape[1]), self.img.shape[1] - 1)
                    y = min(int(landmark.y * self.img.shape[0]), self.img.shape[0] - 1)
                    x_max = max(x_max, x)
                    y_max = max(y_max, y)
                    x_min = min(x_min, x)
                    y_min = min(y_min, y)

                center_x = int((x_max + x_min) / 2)
                center_y = int((y_max + y_min) / 2)

                hand_side = "Right" if center_x < self.img.shape[1] // 2 else "Left"
                cv2.rectangle(self.img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.circle(self.img, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(self.img, hand_side, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # 判定した結果を
if __name__ == '__main__':
    try:
        detect = DetectLRArrowHands()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logdebug("FINISHED DETECTING HANDS.py")
        cv2.destroyAllWindows()