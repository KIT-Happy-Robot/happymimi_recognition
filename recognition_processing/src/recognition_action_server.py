#!/usr/bin/env python
# -*- coding: utf-8 -*

import time
import math
import numpy
import rospy
import rosparam
import actionlib
import smach
from smach_ros import ActionServerWrapper
from geometry_msgs.msg import Twist, Point
# -- Action msg --
from happymimi_recognition_msgs.msg import RecognitionProcessingAction
from happymimi_recognition_msgs.srv import RecognitionCountRequest, RecognitionFindRequest, RecognitionLocalizeRequest

from recognition_tools import RecognitionTools

class MimiControl(object):
    def __init__(self):

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size=1)

        self.twist_value = Twist()

    def angleRotation(self, degree):
        while degree > 180:
            degree = degree - 360
        while degree < -180:
            degree = degree + 360
        angular_speed = 50.0 #[deg/s]
        target_time = abs(1.76899*(degree /angular_speed))  #[s]
        if degree >= 0:
            self.twist_value.angular.z = (angular_speed * 3.14159263 / 180.0) #rad
        elif degree < 0:
            self.twist_value.angular.z = -(angular_speed * 3.14159263 / 180.0) #rad
        rate = rospy.Rate(500)
        start_time = time.time()
        end_time = time.time()
        while end_time - start_time <= target_time:
            self.cmd_vel_pub.publish(self.twist_value)
            end_time = time.time()
            rate.sleep()
        self.twist_value.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_value)

    def moveBase(self, rad_speed):
        for speed_i in range(10):
            self.twist_value.linear.x = rad_speed*0.05*speed_i
            self.twist_value.angular.z = 0
            self.cmd_vel_pub.publish(self.twist_value)
            rospy.sleep(0.1)
        for speed_i in range(10):
            self.twist_value.linear.x = rad_speed*0.05*(10-speed_i)
            self.twist_value.angular.z = 0
            self.cmd_vel_pub.publish(self.twist_value)
            rospy.sleep(0.1)
        self.twist_value.linear.x = 0
        self.twist_value.angular.z = 0
        self.cmd_vel_pub.publish(self.twist_value)


class CheckExistence(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['exist_success', 'exist_failed'],
                             input_keys = ['goal_in', 'bbox_in'],
                             output_keys = ['bbox_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: CheckExistence')

        bb = userdata.bbox_in
        exist_flg = bool(RecognitionTools.countObject(RecognitionCountRequest(target_name), bb).object_num)

        if exist_flg:
            return 'exist_success'
        else:
            return 'exist_failed'
        

class Find(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_success', 'find_faild'],
                             input_keys = [],
                             output_keys = [])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Find')

        bb = userdata.bbox_in
        find_flg = RecognitionTools.findObject(RecognitionFindRequest(target_name)).result

        if find_flg:
            return 'find_success'
        else:
            return 'find_failed'

class Localize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['localize_success', 'localize_faild'],
                             input_keys = [],
                             output_keys = [])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Localize')

        bb = userdata.bbox_in
        localize_request = RecognitionLocalizeRequest()
        localize_request.target_name = target_name
        localize_request.sort_option.data = ''
        localize_request.sort_option.num = 0

        object_centroid = RecognitionTools.localizeObject(localize_request).centroid_point

        if not math.isnan(object_centroid.x):
            return 'localize_success'
        else:
            return 'localize_faild'

class CheckCenter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['check-center_success', 'check-center_faild'],
                             input_keys = [],
                             output_keys = [])

        self.mimi_control = MimiControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: CheckCenter')

        object_centroid = RecognitionTools.localizeObject(localize_request).centroid_point

        object_angle = math.atan2(object_centroid.y, object_centroid.x)/math.pi*180
        if abs(object_angle) < 4.5:
            return 'check-center_success'
        else:
            if abs(object_angle) < 10: object_angle=object_angle/abs(object_angle)*10
            self.mimi_control.angleRotation(object_angle)
            #rospy.sleep(4.0)
            return 'check-center_faild'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['retry'],
                             input_keys = [],
                             output_keys = [])

        self.mimi_control = MimiControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: Move')

        #前後進
        move_count += 1
        move_range = -0.8*(((move_count)%4)/2)+0.4
        mimi_control.moveBase(move_range)
        exist_flg = False
        ###
        return 'retry'


if __name__ == '__main__':
    rospy.init_node('recognition_action_server')
    
    sm_top = smach.StateMachine(outcomes = ['success', 'action_failed', 'preempted'],
                          input_keys = ['goal_message', 'result_message'],
                          output_keys = ['result_message'])

    with sm_top:
        smach.StateMachine.add('', (),
                         transitions = {'':''},
                         remapping = {'':''})

    asw = ActionServerWrapper('/recognition/action', RecognitionProcessingAction,
                              wrapped_container = sm_top,
                              succeeded_outcomes = ['success'],
                              aborted_outcomes = ['action_failed'],
                              preempted_outcomes = ['preempted'],
                              goal_key='action_goal',
                              feedback_key='action_feedback',
                              result_key='action_result')
    asw.run_server()

    rospy.spin()
