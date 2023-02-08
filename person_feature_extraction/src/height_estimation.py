#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
from std_msgs.msg import Float64
from happymimi_msgs.srv import SetFloat, SetFloatResponse
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest

class HeightEstimation(object):
    def __init__(self):
        rospy.Service('/person_feature/height_estimation', SetFloat, self.main)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.position_estimate = rospy.ServiceProxy('/detect/depth', PositionEstimator)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.pose_res = Frame()

    def openPoseCB(self, res):
        self.pose_res = res

    def main(self, _):
        #rospy.wait_for_service('/detect/depth')
        height = SetFloatResponse(data=-1)
        self.head_pub.publish(-25.0)
        rospy.sleep(2.5)
        
        pose = self.pose_res
<<<<<<< HEAD
        if len(pose.persons)==0: return height
=======
        if len(pose.persons)==0: return height#検知しない場合-1を返す

>>>>>>> 305736ddabd432bd203be16115c4a1c70884d532
        #鼻の位置を検出
        center_x = pose.persons[0].bodyParts[0].pixel.y
        center_y = pose.persons[0].bodyParts[0].pixel.x
        if center_x==0 and center_y==0:
<<<<<<< HEAD
            #右目の検出
            center_x = pose.persons[0].bodyParts[15].pixel.y
            center_y = pose.persons[0].bodyParts[15].pixel.x
            if center_x==0 and center_y==0:
                #左目の検出
=======
            #右目の位置の検出
            center_x = pose.persons[0].bodyParts[15].pixel.y
            center_y = pose.persons[0].bodyParts[15].pixel.x
            if center_x==0 and center_y==0:
                #左目の位置の検出
>>>>>>> 305736ddabd432bd203be16115c4a1c70884d532
                center_x = pose.persons[0].bodyParts[16].pixel.y
                center_y = pose.persons[0].bodyParts[16].pixel.x
                #身長の推定がうまく行きそうにない場合、再度推定を行う
                if center_x==0 and center_y==0:
                    return height
<<<<<<< HEAD
                
        #if center_x<0: return height
        #if center_x>479: return height
        #if center_y<0: return height
        #if center_y>639: return height
        
        print(center_x,center_y)
        #print(lheel_x,lheel_y)
        #print(lheel_x - center_x)
=======
        
        #身長の推定がうまく行きそうにない場合、再度推定を行う
        if center_x<0: center_x=0 return height
        if center_x>479: center_x=479 return height
        if center_y<0: center_y=0 return height
        if center_y>639: center_y=639 return height


>>>>>>> 305736ddabd432bd203be16115c4a1c70884d532
        rospy.wait_for_service('/detect/depth')
        p_e_req = PositionEstimatorRequest()
        p_e_req.center_x = int(center_x)
        p_e_req.center_y = int(center_y)
        p_e_res = self.position_estimate(p_e_req).point

        height.data = p_e_res.z*100 + 30
        print(p_e_res.z*100) 
        return height

if __name__ == '__main__':
    rospy.init_node('height_estimation')
    height_estimation = HeightEstimation()
    rospy.spin()
