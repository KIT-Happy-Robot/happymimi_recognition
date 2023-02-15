#!/usr/bin/env python3
#-*- coding: utf-8-*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
from std_msgs.msg import Float64
from happymimi_msgs.srv import SetStr, SetFloat, SetFloatResponse, SimpleTrg, SimpleTrgResponse
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest

class FallingDown(object):
    def __init__(self):
        rospy.Service('/person_feature/falling_down_person', SetFloat, self.main)
        rospy.Service('/person_feature/falling_down_recognition', SimpleTrg, self.main)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.position_estimate = rospy.ServiceProxy('/detect/depth', PositionEstimator)
        #self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)

        self.pose_res = Frame()

    def openPoseCB(self, res):
        self.pose_res = res

    def main(self, req):
        fall = SimpleTrgResponse()
        fall = False
        #self.head_pub.publish(-25.0)
        rospy.sleep(2.5)
        #rospy.loginfo("test") #デバッグ用

        pose = self.pose_res
        rospy.loginfo(pose.persons)
        if len(pose.persons)==0: return fall

        #頭の位置を検出
        center_x_head = pose.persons[0].bodyParts[0].pixel.y
        center_y_head = pose.persons[0].bodyParts[0].pixel.x
        print(center_x_head, center_y_head)

        if (center_x_head == 0) and (center_y_head==0):
            fall = False

        else:
            fall = True
        #右腰の位置の検出
        #center_x_rhip = pose.persons[0].bodyParts[9].pixel.y
        #center_y_rhip = pose.persons[0].bodyParts[9].pixel.x
        #print(center_x_rhip, center_y_rhip) 

        #if (center_x_rhip==0) and (center_y_rhip==0):
        #    #左腰の位置の検出
        #    center_x_lhip = pose.persons[0].bodyParts[12].pixel.y
        #    center_y_lhip = pose.persons[0].bodyParts[12].pixel.x
        #    print(center_x_lhip, ceter_y_lhip)


        rospy.wait_for_service('/detect/depth')
        head_z_req = PositionEstimatorRequest()
        head_z_req.center_x = int(center_x_head)
        head_z_req.center_y = int(center_y_head)
        head_z_res = self.position_estimate(head_z_req).point
        print(head_z_res)
        

        return fall 


if __name__ == '__main__':
    rospy.init_node('Falling_Down')
    FD = FallingDown()
    rospy.spin()
