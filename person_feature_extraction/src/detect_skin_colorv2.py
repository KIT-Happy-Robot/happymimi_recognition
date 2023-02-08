#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import cv2
import collections
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from ros_openpose.msg import Frame
from cv_bridge import CvBridge, CvBridgeError
from happymimi_msgs.srv import SetStr, SetStrResponse

class DetectClothColor(object):
    def __init__(self):
        rospy.Service('/person_feature/skin_color', SetStr, self.main)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        #実験用に首が動かないようにしている
        #self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.image_res = Image()
        self.pose_res = Frame()

    def realsenseCB(self, res):
        self.image_res = res

    def openPoseCB(self, res):
        self.pose_res = res

    def judgeColor(self, req):
        # hsv色空間で色の判定
        h, s, v = req
        #h,s,v = req
        print( h, s, v)
        color = ''
        #print(req)

        '''
        if 0<=v and v<=79: color = 'Black'
        #if 0<=v and v<=50: color = 'Black'
        if (25<=h and h<= 40) and (20<=s and s<=35): color = 'skin'
        if (0<=s and s<=25) and (190<=v and v<=255): color = 'White'#→Europe
        if (0<=s and s<=50) and (80<=v and v<=130): color = 'Gray'
        #elif (50 <= s and s <= 170) and (70 <= v and v <= 150): color = 'Gray'
        #elif (50<=s and s<=170) and (80<=v and v<=90): color = 'Gray'
        #elif (0<=s and s<=50) and (80<=v and v<=230): color = 'Gray'
        #elif (5<=h and h<=18) and (20<=s and s<=240) and (70<=v and v<=180): color = 'Brown'
        elif (5<=h and h<=18) and v<=200: color = 'Brown' #→Asia?
        elif (0<=h and h<=4) or (174<=h and h<=180): color = 'Red'
        elif 5<=h and h<=18: color = 'Orange'
        elif 20<=h and h<=59: color = 'Yellow'
        elif 60<=h and h<=89: color = 'Green'
        elif 180<=h and h<=240: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 160<=h and h<=173: color = 'Pink'
        '''
        if 0<= v and v<= 50: color = 'Black'
        if 200<= v and v <= 255 : color = 'White'
        if 105 <= v and 199 <= v : color = 'Gray'
        if (110<=h and h<=130) and (120<=s and s<=160): color = 'Brown' #黒人
        elif 110<=h and h<=130: color = 'Red'
        if 100<=h and h<=110: color = 'Orange' #Asia
        if (25<=h and h<= 40) and (20<=s and s<=35): color = 'skin'
        if 75<=h and h<=99: color = 'Yellow'#Asia
        elif 50<=h and h<=74: color = 'Green'
        elif 0<=h and h<=20: color = 'Blue'
        elif 137<=h and h<=159: color = 'Purple'
        elif 120<=h and h<=135: color = 'Pink' #白人?
        return color

    def main(self, _):
        response = SetStrResponse()

        #self.head_pub.publish(-20.0)
        #rospy.sleep(2.5)

        pose = self.pose_res
        if len(pose.persons)==0: return response

        # neck, REye,LEye,Noseの位置を取得
        neck_x = pose.persons[0].bodyParts[1].pixel.y
        neck_y = pose.persons[0].bodyParts[1].pixel.x
        reye_x = pose.persons[0].bodyParts[15].pixel.y
        reye_y = pose.persons[0].bodyParts[15].pixel.x
        leye_x = pose.persons[0].bodyParts[16].pixel.y
        leye_y = pose.persons[0].bodyParts[16].pixel.x
        nose_x = pose.persons[0].bodyParts[0].pixel.y
        nose_y = pose.persons[0].bodyParts[0].pixel.x
        
        rear_x = pose.persons[0].bodyParts[17].pixel.y
        rear_y = pose.persons[0].bodyParts[17].pixel.x
        lear_x = pose.persons[0].bodyParts[18].pixel.y
        lear_y = pose.persons[0].bodyParts[17].pixel.x                           
        
        print('neck: ', neck_x, neck_y)
        print('reye: ', reye_x, reye_y)
        print('leye: ', leye_x, leye_y)
        print('nose: ', nose_x, nose_y)
        print("lear", lear_x,lear_y)
        print("rear", rear_x, rear_y)
        
        """ if (neck_x==0.0 and neck_y==0.0) and (nose_x==0.0 and nose_y==0.0):
            return response
        elif nose_x==0.0 and nose_y==0.0:
            face_axis_x = (reye_x + leye_x) / 2.0
            face_axis_y = (reye_y + leye_y) / 2.0
        else:
            face_axis_x = nose_x
            face_axis_y = nose_y
            if nose_x==0.0 and nose_y==0.0:
                face_length = int(479 - neck_x)
            else:
                face_length = int(nose_x  - neck_x)
            
        if face_axis_x<0: face_axis_x=0
        if face_axis_x>479: face_axis_x=479
        if face_axis_y<0: face_axis_y=0
        if face_axis_y>639: face_axis_y=639

        if (rear_x==0.0 and rear_y==0.0) and (lear_x==0.0 and lear_y==0.0):
            pass
        elif rear_x==0.0 and rear_y==0.0:
            width = int(lear_y - face_axis_y)
        elif lear_x==0.0 and lear_y==0.0:
            width = int(face_axis_y - rear_y)
        else:
            ear_width = lear_y - rear_y
            width = int(ear_width/2)
 """    
        face_length = int(neck_x - nose_x - 50)
        face_axis_x = int(nose_x)
        face_axis_y = int(nose_y)
        width = int(rear_x - lear_x)
        if width < 0:
            width = -1*width
        
        
        print("face_length:", face_length)
        print("face_axis_x:", face_axis_x)
        print("face_axis_y:", face_axis_y)
        print("width:", width)
        
        # 画像の変換
        image = CvBridge().imgmsg_to_cv2(self.image_res)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_map = ['']
        for i in range(face_length):
            x = i + face_axis_x
            #if x<0 or x>479: continue
            for j in range(width):
                y = j + face_axis_y
                #if y<0 or y>639: continue
                color = self.judgeColor(hsv_image[int(x), int(y)])
                color_map.append(color)
        print(color_map)
        count_l = collections.Counter(color_map)
        response.result = count_l.most_common()[0][0]
        
        print(response.result)
        return response

if __name__ == '__main__':
    rospy.init_node('detect_skin_color')
    detect_cloth_color = DetectClothColor()
    rospy.spin()
