#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
from happymimi_msgs.srv import SetFloat
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
        if len(pose.persons)==0: return -1.0

        print pose.persons[0].bodyParts[0].pixel
        center_x = pose.persons[0].bodyParts[0].pixel.x
        center_y = pose.persons[0].bodyParts[0].pixel.y
        print center_x, center_y
        if center_x==0 and center_y==0: return -1.0

        rospy.wait_for_service('/detect/depth')
        p_e_req = PositionEstimatorRequest()
        p_e_req.center_x = int(center_x)
        p_e_req.center_y = int(center_y)
        p_e_res = self.position_estimate(p_e_req).point

        print 'z:', p_e_res.z
        height = p_e_res.z + 30
        return height

if __name__ == '__main__':
    rospy.init_node('height_estimation')

    rospy.sleep(0.5)
    h_e = HeightEstimation()
    rospy.sleep(2.0)
    a = h_e.main()
    print a
    rospy.spin()
