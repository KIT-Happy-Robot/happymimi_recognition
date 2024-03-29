#!/usr/bin/env python3
#-*- coding: utf-8-*

import rospy
from ros_openpose.msg import AltMarkerArray, Frame
from std_msgs.msg import Float64
<<<<<<< HEAD

from happymimi_msgs.srv import SetStr ,SetFloat

from happymimi_msgs.srv import SetStr, SetFloat

from happymimi_msgs.srv import SetStr, SetFloat, SetFloatResponse, SimpleTrg

=======
from happymimi_msgs.srv import SetStr, SetFloat, SetFloatResponse, SimpleTrg, SimpleTrgResponse
>>>>>>> 9328f4628cf6de0d3df46ee9593bddfaa7a95fef
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

<<<<<<< HEAD
    def main(self, _):
        fall = SetFloatResponse(data=-1)
=======
    def main(self, req):
        fall = SimpleTrgResponse()
        fall = False
>>>>>>> 9328f4628cf6de0d3df46ee9593bddfaa7a95fef
        #self.head_pub.publish(-25.0)
        rospy.sleep(2.5)
        #rospy.loginfo("test") #デバッグ用

        pose = self.pose_res
        rospy.loginfo(pose.persons)
        if len(pose.persons)==0: return fall

        #頭の位置を検出
<<<<<<< HEAD
        center_x = pose.persons[0].bodyParts[0].pixel.y
        center_y = pose.persons[0].bodyParts[0].pixel.x
=======
        center_x_head = pose.persons[0].bodyParts[0].pixel.y
        center_y_head = pose.persons[0].bodyParts[0].pixel.x
<<<<<<< HEAD
        print(center_x_head, center_y_head)
>>>>>>> 9328f4628cf6de0d3df46ee9593bddfaa7a95fef
=======
        #print(center_x_head, center_y_head)
>>>>>>> 6497171400f6dd96d2723fe32931fc628c5b6886

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

<<<<<<< HEAD
            if center_x==0 and center_y==0:
                #左耳の位置の検出
                center_x = pose.persons[0].bodyParts[18].pixel.y
                center_y = pose.persons[0].bodyParts[18].pixel.x
                #推定がうまく行かなかった場合再度推定を行う
                if center_x==0 and center_y==0:
                    return fall
        
        print(center_x, center_y)
=======

>>>>>>> 9328f4628cf6de0d3df46ee9593bddfaa7a95fef
        rospy.wait_for_service('/detect/depth')
        head_z_req = PositionEstimatorRequest()
        head_z_req.center_x = int(center_x_head)
        head_z_req.center_y = int(center_y_head)
        head_z_res = self.position_estimate(head_z_req).point
        print(head_z_res.z*100)

        return fall 


if __name__ == '__main__':
    rospy.init_node('Falling_Down')
    FD = FallingDown()
    rospy.spin()
