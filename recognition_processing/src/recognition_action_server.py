#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import rosparam
import roslib.packages
import os
import sys
import time
import math
import numpy
import actionlib
import smach
from smach_ros import ActionServerWrapper
from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes
from happymimi_msgs.msg import StrInt
from happymimi_recognition_msgs.msg import RecognitionProcessingAction, RecognitionProcessingResult
from happymimi_recognition_msgs.srv import RecognitionCountRequest, RecognitionFindRequest, RecognitionLocalizeRequest


from recognition_tools import RecognitionTools
teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl


class Server(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_action'],
                             input_keys = ['goal_in'],
                             output_keys = ['target_name_out', 'sort_option_out', 'e_l_count_out', 'c_l_count_out', 'move_count_out', 'result_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Server')

        userdata.target_name_out = userdata.goal_in.target_name
        userdata.sort_option_out = userdata.goal_in.sort_option
        userdata.e_l_count_out = 0
        userdata.c_l_count_out = 0
        userdata.move_count_out = 0
        userdata.result_out = RecognitionProcessingResult(result_flg=False, centroid_point=Point())
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
        localize_request.target_name = userdata.target_name_in
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

        self.base_control = BaseControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: CheckCenter')

        object_angle = math.atan2(userdata.centroid_in.y, userdata.centroid_in.x)/math.pi*180
        reset_option = StrInt(data='center', num=0)
        userdata.sort_option_out = reset_option

        if abs(object_angle) < 4.5:
            userdata.result_out = RecognitionProcessingResult(result_flg=True, centroid_point=userdata.centroid_in)
            return 'check_center_success'
        elif userdata.c_l_count_in > 3:
            return 'action_failed'
        else:
            if abs(object_angle) < 10: object_angle=object_angle/abs(object_angle)*10
            self.base_control.rotateAngle(object_angle)
            #rospy.sleep(4.0)
            userdata.c_l_count_out = userdata.c_l_count_in + 1
            return 'check_center_failure'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['retry'],
                             input_keys = ['move_count_in'],
                             output_keys = ['move_count_out'])

        self.base_control = BaseControl()

    def execute(self, userdata):
        rospy.loginfo('Executing state: Move')

        move_range = 0.4*(((userdata.move_count_in+1)%4)/2)-0.2
        self.base_control.translateDist(move_range)
        userdata.move_count_out += 1
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
                                      'c_l_count_out':'center_loop_count',
                                      'move_count_out':'move_count',
                                      'result_out':'action_result'})

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
                         remapping = {'move_count_in':'move_count',
                                      'move_count_out':'move_count'})


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
