#!/usr/bin/env python3
#-*- coding: utf-8-*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
from std_msgs.msg import Float64
from happymimi_msgs.srv import SetStr, SetFloat, SetFloatResponse, SimpleTrg.srv
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest

class Falling_Down(object):
    def __init__(self):
        rospy.Service('/person_feature/falling_down_recognition', SimpleTrg, self.main)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.position_estimate = rospy.ServiceProxy('/detect/depth', PositionEstimator)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.pose_res = Frame()

    def openPoseCB(self, res):
        self.pose_res = res

    def main(self, _):
        fall = SimpleTrg.srv(data=0)
        self.head_pub.publish(-25.0)
        rospy.sleep(2.5)
        rospy.loginfo("test") #デバッグ用

        pose = self.pose_res
        rospy.loginfo(int(pose.persons))
        if len(pose.persons)==0: return fall
        
        #頭の位置を検出
        center_x_head = pose.persons[0].bodyParts[0].pixel.y
        center_y_head = pose.persons[0].bodyParts[0].pixel.x
        rospy.loginfo(center_x_head, center_y_head)

        #右腰の位置の検出
        center_x_rhip = pose.persons[0].bodyParts[9].pixel.y
        center_y_rhip = pose.persons[0].bodyParts[9].pixel.x
        rospy.loginfo(center_x_rhip, center_y_rhip) 

        if (center_x_rhip==0) and (center_y_rhip==0):
            #左腰の位置の検出
            center_x_lhip = pose.persons[0].bodyParts[12].pixel.y
            center_y_lhip = pose.persons[0].bodyParts[12].pixel.x
            rospy.loginfo(center_x_lhip, center_y_lhip)


        

        rospy.wait_for_service('/detect/depth')

        return fall 


if __name__ == '__main__':
    rospy.init_node('Falling_Down')
    falling_down = Falling_Down()
    rospy.spin()
