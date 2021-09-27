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
from darknet_ros_msgs.msg import BoundingBoxes
from happymimi_msgs.msg import StrInt
# -- Action msg --
from happymimi_recognition_msgs.msg import RecognitionProcessingAction
from happymimi_recognition_msgs.srv import RecognitionCountRequest, RecognitionFindRequest, RecognitionLocalizeRequest
# -- Class import --
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


class Server(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_action'],
                             input_keys = ['goal_in'],
                             output_keys = ['target_name_out', 'sort_option_out', 'e_l_count_out', 'c_l_count_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Server')

        userdata.target_name_out = userdata.goal_in.target_name
        userdata.sort_option_out = userdata.goal_in.sort_option
        userdata.e_l_count_out = 0
        userdata.c_l_count_out = 0
        return 'start_action'

class Count(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['count_success', 'count_failure', 'action_failed'],
                             input_keys = ['target_name_in', 'sort_option_in', 'e_l_count_in'],
                             output_keys = ['sort_option_out', 'bbox_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Count')

        bbox = Recognition_Tools.bbox
        userdata.bbox_out = bbox
        object_count = Recognition_Tools.countObject(RecognitionCountRequest(userdata.target_name_in), bbox).object_num
        exist_flg = object_count > 0

        if (userdata.sort_option_in.num + 1) > object_count:
            reset_option = StrInt(data='center', num=0)
            userdata.sort_option_out = reset_option

        if exist_flg:
            return 'count_success'
        elif userdata.e_l_count_in > 3:
            return 'action_failed'
        else:
            return 'count_failure'

class Find(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_success', 'find_failure'],
                             input_keys = ['target_name_in', 'e_l_count_in'],
                             output_keys = ['e_l_count_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Find')

        find_flg = Recognition_Tools.findObject(RecognitionFindRequest(userdata.target_name_in)).result

        if find_flg:
            return 'find_success'
        else:
            userdata.e_l_count_out = userdata.e_l_count_in + 2
            return 'find_failure'

class Localize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['localize_success', 'localize_failure'],
                             input_keys = ['target_name_in', 'sort_option_in', 'bbox_in', 'e_l_count_in'],
                             output_keys = ['centroid_out', 'e_l_count_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Localize')

        localize_request = RecognitionLocalizeRequest()
        localize_request.target_name = target_name
        localize_request.sort_option = userdata.sort_option_in
        object_centroid = Recognition_Tools.localizeObject(localize_request).centroid_point
        userdata.centroid_out = object_centroid

        if not math.isnan(object_centroid.x):
            return 'localize_success'
        else:
            userdata.e_l_count_out = userdata.e_l_count_in + 1
            return 'localize_failure'

class CheckCenter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['check_center_success', 'check_center_failure', 'action_failed'],
                             input_keys = ['sort_option_in', 'centroid_in', 'c_l_count_in'],
                             output_keys = ['sort_option_out', 'c_l_count_out', 'result_out'])

        self.mimi_control = MimiControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: CheckCenter')

        object_angle = math.atan2(userdata.centroid_in.y, userdata.centroid_in.x)/math.pi*180
        reset_option = StrInt(data='center', num=0)
        userdata.sort_option_out = reset_option

        if abs(object_angle) < 4.5:
            userdata.result_out.result_flg = True
            userdata.result_out.centroid_point = userdata.centroid_in
            return 'check_center_success'
        elif c_l_count_in > 3:
            return 'action_failed'
        else:
            if abs(object_angle) < 10: object_angle=object_angle/abs(object_angle)*10
            self.mimi_control.angleRotation(object_angle)
            #rospy.sleep(4.0)
            c_l_count_out = c_l_count_in + 1
            return 'check_center_failure'

class Move(smach.State):
    move_count = 0

    def __init__(self):
        smach.State.__init__(self, outcomes = ['retry'],
                             input_keys = [],
                             output_keys = [])

        self.mimi_control = MimiControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: Move')

        '''
        Move.move_count += 1
        move_range = -0.8*(((move_count)%4)/2)+0.4
        mimi_control.moveBase(move_range)
        '''
        return 'retry'


if __name__ == '__main__':
    rospy.init_node('recognition_action_server')

    global Recognition_Tools
    Recognition_Tools = RecognitionTools()

    sm= smach.StateMachine(outcomes = ['success', 'action_failed', 'preempted'],
                          input_keys = ['action_goal', 'action_result'],
                          output_keys = ['action_result'])

    with sm:
        smach.StateMachine.add('SERVER', Server(),
                         transitions = {'start_action':'COUNT'},
                         remapping = {'goal_in':'action_goal',
                                      'target_name_out':'target_name',
                                      'sort_option_out':'sort_option',
                                      'e_l_count_out':'existence_loop_count',
                                      'c_l_count_out':'center_loop_count'})

        smach.StateMachine.add('COUNT', Count(),
                         transitions = {'count_success':'LOCALIZE',
                                        'count_failure':'FIND',
                                        'action_failed':'action_failed'},
                         remapping = {'target_name_in':'target_name',
                                      'sort_option_in':'sort_option',
                                      'e_l_count_in':'existence_loop_count',
                                      'sort_option_out':'sort_option',
                                      'bbox_out':'bbox'})

        smach.StateMachine.add('FIND', Find(),
                         transitions = {'find_success':'LOCALIZE',
                                        'find_failure':'MOVE'},
                         remapping = {'target_name_in':'target_name',
                                      'e_l_count_in':'existence_loop_count',
                                      'e_l_count_out':'existence_loop_count'})

        smach.StateMachine.add('LOCALIZE', Localize(),
                         transitions = {'localize_success':'CHECK_CENTER',
                                        'localize_failure':'MOVE'},
                         remapping = {'target_name_in':'target_name',
                                      'sort_option_in':'sort_option',
                                      'bbox_in':'bbox',
                                      'e_l_count_in':'existence_loop_count',
                                      'centroid_out':'centroid',
                                      'e_l_count_out':'existence_loop_count'})

        smach.StateMachine.add('CHECK_CENTER', CheckCenter(),
                         transitions = {'check_center_success':'success',
                                        'check_center_failure':'LOCALIZE',
                                        'action_failed':'action_failed'},
                         remapping = {'sort_option_in':'sort_option',
                                      'centroid_in':'centroid',
                                      'c_l_count_in':'center_loop_count',
                                      'sort_option_out':'sort_option',
                                      'c_l_count_out':'center_loop_count',
                                      'result_out':'action_result'})

        smach.StateMachine.add('MOVE', Move(),
                         transitions = {'retry':'COUNT'},
                         remapping = {})


    asw = ActionServerWrapper('/recognition/action', RecognitionProcessingAction,
                              wrapped_container = sm,
                              succeeded_outcomes = ['success'],
                              aborted_outcomes = ['action_failed'],
                              preempted_outcomes = ['preempted'],
                              goal_key='action_goal',
                              feedback_key='action_feedback',
                              result_key='action_result')
    asw.run_server()

    rospy.spin()
