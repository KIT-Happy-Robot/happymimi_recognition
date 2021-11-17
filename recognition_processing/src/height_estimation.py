#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
#from happymimi_msgs.srv import SetFloat
from happymimi_recognition_msgs.srv import PositionEstimator, PositionEstimatorRequest

class HeightEstimation(object):
    def __init__(self):
        rospy.Service('/height_estimation', SetFloat, self.main)
        #rospy.Subscriber('/visualization', AltMarkerArray, self.openPoseCB)
        rospy.Subscriber('/frame', Frame, self.openPoseCB)
        self.position_estimate = rospy.ServiceProxy('/detect/depth', PositionEstimator)

        self.pose_res = Frame()

    def openPoseCB(self, res):
        self.pose_res = res

    def main(self):
        pose = self.pose_res
        if len(pose.persons)==0: return 170.0

        center_x = center_y = pose.person[0].bodyPart[0]
        if center_x==0 and center_y==0: return 170.0

        rospy.wait_for_service('/detect/depth')
        p_e_req = PositionEstimatorRequest()
        p_e_req.center_x = center_x
        p_e_req.center_y = center_y
        p_e_res = self.detect_depth(p_e_req).point

        height = p_e_res.z + 30
        return height

if __name__ == '__main__':
    rospy.init_node('height_estimation')

    rospy.sleep(0.5)
    HeightEstimation()
    rospy.spin()
