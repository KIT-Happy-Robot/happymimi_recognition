#!/usr/bin/env python3

from MotionBool.srv import *
import cv2
import rospy
import sys

<<<<<<< HEAD
class detection():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  #カメラの設定（パソコン内蔵カメラは「0」）
        self.before = None              #画像の比較（1つ前の画像）
        self.colour = (0,0,255)         #線の色（赤）

    def motion_detection_server(self):
        rospy.init_node('motion_deteciton_sever')
        s = rospy.Service('motion_dt',MotionBool,self.motion)
        print('Ready to motion detection')
        rospy.spin()
    
    def motion(self,req):
        res = MotionBoolResponse()
        count = 0
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
                    res.result = False
                    continue
                else:
                    res.result = True
                cv2.rectangle(frame,(x,y),(x+w,y+h),self.colour,2)
                count += 1
                
                
                print('検知しました{}'.format(count))
                if req.count > 20:
                    print('result:{}'.format(res.result))
                    cv2.destroyAllWindows()
                    return res.result

            #ウィンドウで表示
            cv2.imshow('target_frame',frame)
            #cv2.waitKey(1)

if __name__ == '__main__':
    d = detection()
    d.motion_detection_server()
=======

cap = cv2.VideoCapture(0)

def motion_detection_server():
    rospy.Service('motion_dt',MotionBool, motion)
    print('Ready to motion detection')

def cap_imshow(img):
    cv2.imshow('motion_detection_frame',img)
    cv2.waitKey(1)

def motion(req):
    before = None
    res = MotionBoolResponse()
    count = 0
    colour = (0,0,255) 
    flg = False

    while True:
        ret,frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        if before is None:
            before = gray.astype('float')
            continue

        #現在のフレームと移動平均との差を計算
        cv2.accumulateWeighted(gray, before, 0.88)
        frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(before))
        #frameDeltaの画像を２値化
        thresh = cv2.threshold(frameDelta, 3 , 255, cv2.THRESH_BINARY)[1]
        #輪郭のデータを得る
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        #差分があった点を画面に描く
        for target in contours:
            x,y,w,h = cv2.boundingRect(target) 
            if w < 100:
                flg = False
                continue
            else:
                flg = True
                res.result = True
            cv2.rectangle(frame,(x,y),(x+w,y+h),colour,2)       
        if flg:
            count += 1
            print('検知しました{}'.format(count))
            if count >= 10:
                print('result:{}'.format(res.result))
                #cap.release()
                #cv2.destroyAllWindows()
                return res.result
        
        #ウィンドウで表示
        #cap_imshow(frame)
     

if __name__ == '__main__':
    rospy.init_node('motion_deteciton_sever')
    motion_detection_server()
    rospy.spin()
>>>>>>> aa3518ae8f35e73e3955e668e0995c5253ea28ca
