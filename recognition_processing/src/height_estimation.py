#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
from ros_openpose.msg import AltMarkerArray
from happymimi_msgs.srv import SetFloat
from happymimi_recognition_msgs.srv import RecognitionLocalize, RecognitionLocalizeRequest

class HeightEstimation(object):
    def __init__(self):
        rospy.Service('/height_estimation', SetFloat, self.main)
        rospy.Subscriber('/visualization', AltMarkerArray, self.openPoseCB)

        self.pose_res = AltMarkerArray()

    def openPoseCB(self, res):
        self.pose_res = res
        print res

    def main(self):
        self.pose_res.markers[0]
        rospy.wait_for_service('/recognition/localize')
        req = RecognitionLocalizeRequest()
        req.target_name = 'person'
        req.sort_option.data = 'center'
        req.sort_option.num = 0
        res = self.localize_func(req).point

if __name__ == '__main__':
    rospy.init_node('height_estimation')
    
    rospy.sleep(0.5)
    HeightEstimation()
    rospy.spin()
