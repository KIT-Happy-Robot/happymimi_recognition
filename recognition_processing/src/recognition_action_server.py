#!/usr/bin/env python
# -*- coding: utf-8 -*

import time
import math
import numpy
import rospy
import rosparam
import actionlib
from geometry_msgs.msg import Twist, Point
# -- Action msg --
from happymimi_recognition_msgs.msg import *

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


class RecognitionActionServer(object):
    def __init__(self):
        self.act = actionlib.SimpleActionServer('/manipulation/localize',
                                                RecognitionProcessingAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)

        self.preempt_flg = False
        self.act.start()

        self.recognition_tools = RecognitionTools()
        self.recognition_tools.initializeBBox()

    def actionPreempt(self):
        rospy.loginfo('preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self, goal):
        target_name = goal.recognition_goal
        rospy.loginfo('start action >> recognize %s'%(target_name))
        action_feedback = RecognitionProcessingFeedback()
        action_result = RecognitionProcessingResult()
        mimi_control = MimiControl()
        move_count = 0
        while not rospy.is_shutdown():
            bb = self.recognition_tools.bbox
            object_count, _ = self.recognition_tools.countObject(target_name, bb)
            exist_flg = bool(object_count)
            if exist_flg:
                object_centroid = self.recognition_tools.localizeObject(target_name, bb)
                if not math.isnan(object_centroid.x):# 物体が正面になるように回転する処理
                    object_angle = math.atan2(object_centroid.y, object_centroid.x)/math.pi*180
                    if abs(object_angle) < 4.5:
                        # success
                        rospy.loginfo('Succeeded')
                        action_result.recognition_result = object_centroid
                        self.act.set_succeeded(action_result)
                        break
                    else:
                        # retry
                        rospy.loginfo('There is not object in front.')
                        object_angle *= 1.5 # kobukiが重いので
                        if abs(object_angle) < 10: object_angle=object_angle/abs(object_angle)*10
                        mimi_control.angleRotation(object_angle)
                        rospy.sleep(4.0)
                else:
                    #前後進
                    move_count += 1
                    move_range = -0.8*(((move_count)%4)/2)+0.4
                    mimi_control.moveBase(move_range)
                    exist_flg = False
            else:
                find_flg = self.recognition_tools.findObject(target_name)
                exist_flg = find_flg
            action_feedback.recognition_feedback = exist_flg
            self.act.publish_feedback(action_feedback)
            rospy.sleep(1.0) #preemptのズレ調整用
            if self.preempt_flg:
                self.preempt_flg = False
                break


if __name__ == '__main__':
    rospy.init_node('recognition_action_server')
    recognition_action_server = RecognitionActionServer()
    rospy.spin()
