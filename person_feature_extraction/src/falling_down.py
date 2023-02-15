#!/usr/bin/env python3
#-*- coding: utf-8-*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
from std_msgs.msg import Float64

from happymimi_msgs.srv import SetStr ,SetFloat

from happymimi_msgs.srv import SetStr, SetFloat

from happymimi_msgs.srv import SetStr, SetFloat, SetFloatResponse, SimpleTrg

from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest

class Falling_Down(object):
    def __init__(self):
        rospy.Service('/person_feature/falling_down_recognition', SetFloat, self.main)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.position_estimate = rospy.ServiceProxy('/detect/depth', PositionEstimator)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.pose_res = Frame()

    def openPoseCB(self, res):
        self.pose_res = res

    def main(self, _):
        fall = SetFloatResponse(data=-1)
        #self.head_pub.publish(-25.0)
        rospy.sleep(2.5)
        rospy.loginfo("test") #デバッグ用

        pose = self.pose_res
        if len(pose.persons)==0: return fall

        #頭の位置を検出
        center_x = pose.persons[0].bodyParts[0].pixel.y
        center_y = pose.persons[0].bodyParts[0].pixel.x

        if center_x==0 and center_y==0:
            #右耳の位置の検出
            center_x = pose.persons[0].bodyParts[17].pixel.y
            center_y = pose.persons[0].bodyParts[17].pixel.x

            if center_x==0 and center_y==0:
                #左耳の位置の検出
                center_x = pose.persons[0].bodyParts[18].pixel.y
                center_y = pose.persons[0].bodyParts[18].pixel.x
                #推定がうまく行かなかった場合再度推定を行う
                if center_x==0 and center_y==0:
                    return fall
        
        print(center_x, center_y)
        rospy.wait_for_service('/detect/depth')
        p_e_req = PositionEstimatorRequest()
        p_e_req.center_x = int(center_x)
        p_e_req.center_y = int(center_y)
        p_e_res = self.position_estimate(p_e_req).point

        height.data = p_e_rest.z*100 + 30
        print(p_e_rst.z*100)
        return fall 


if __name__ == '__main__':
    rospy.init_node('Falling_Down')
    falling_down = Falling_Down()
    rospy.spin()
