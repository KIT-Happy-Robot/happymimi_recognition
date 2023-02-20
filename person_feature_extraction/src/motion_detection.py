#!/usr/bin/env python3

from mimi_motion_detection.srv import *
import cv2
import rospy
import sys

class ditection():
    def __init__(self):
        motion_dt = rospy.ServiceProxy('motion_dt',MotionBool,self.motion)
        self.cap = cv2.VideoCapture(0)  #カメラの設定（パソコン内蔵カメラは「0」）
        self.before = None              #画像の比較（1つ前の画像）
        self.count = 0                  #テスト用カウント変数
        self.colour = (0,0,255)         #線の色（赤）

    #main関数
    def motion(self):
        res = MotionBoolResponse()
        rospy.wait_for_service('motion_dt')
        try:
            while True:
                ret,frame = self.cap.read()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                

                if self.before is None:
                    self.before = gray.astype('float')
                    continue

                #現在のフレームと移動平均との差を計算
                cv2.accumulateWeighted(gray, self.before, 0.88)
                frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.before))
                #frameDeltaの画像を２値化
                thresh = cv2.threshold(frameDelta, 3 , 255, cv2.THRESH_BINARY)[1]
                #輪郭のデータを得る
                contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
                
                #差分があった点を画面に描く
                for target in contours:
                    x,y,w,h = cv2.boundingRect(target)
                    if w < 100:
                        continue
                    cv2.rectangle(frame,(x,y),(x+w,y+h),self.colour,2)
                    self.count += 1
                    if count > 20:
                        print('検知しました{}'.format(self.count))
                        return res.result == True


                #ウィンドウで表示
                cv2.imshow('target_frame',frame)
                #Enterキーが押されたらループを抜ける
                if cv2.waitKey(1) == 13:
                    break

            #ウィンドウを破棄
            cv2.destroyAllWindows()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == '__main__':
    d = ditection()
    d.motion()
