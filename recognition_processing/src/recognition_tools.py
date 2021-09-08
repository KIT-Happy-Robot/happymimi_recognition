#!/usr/bin/env python
# -*- coding: utf-8 -*

import time
import numpy
import rospy
import rosparam
import rosgraph
from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes
# -- Custom Message --
from happymimi_recognition_msgs.srv import RecognitionList, RecognitionListResponse, RecognitionCount, RecognitionFind, RecognitionLocalize, PositionEstimator

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


class CallDetector(object):
    def __init__(self):
        self.detect_depth = rospy.ServiceProxy('/detect/depth', PositionEstimator)

        self.object_centroid = Point()

    def detectorService(self, center_x, center_y):
        rospy.wait_for_service('/detect/depth')
        res = self.detect_depth(center_x, center_y)
        self.object_centroid = res.centroid_point
        rospy.loginfo(self.object_centroid)


class RecognitionTools(object):
    def __init__(self):
        rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.boundingBoxCB)
        rospy.Service('/recognition/list',RecognitionList,self.listObject)
        rospy.Service('/recognition/find',RecognitionFind,self.findObject)
        rospy.Service('/recognition/count',RecognitionCount,self.countObject)
        rospy.Service('/recognition/localize',RecognitionLocalize,self.localizeObject)

        self.image_height = 480# rosparam.get_param('/camera/realsense2_camera/color_height')
        self.image_width = 640# rosparam.get_param('/camera/realsense2_camera/color_width')
        try:
            self.object_dict = rosparam.get_param('/object_dict')
        except rosgraph.masterapi.MasterError:
            self.object_dict = {'any':['cup', 'bottle']}

        self.bbox = []
        self.update_time = 0 # darknetからpublishされた時刻を記録
        self.update_flg = False # darknetからpublishされたかどうかの確認

        rospy.Timer(rospy.Duration(0.5), self.initializeBbox)

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.bounding_boxes

    def initializeBbox(self, event):
        # darknetが何も認識していない時にself.bboxを初期化する
        if time.time() - self.update_time > 1.0 and self.update_flg:
            self.bbox = []
            self.update_flg = False
            rospy.loginfo('initialize')

    def createBboxList(self,bb):
        bbox_list = []
        for i in range(len(bb)):
            bbox_list.append(bb[i].Class)
        return bbox_list

    def listObject(self, request, bb=None, internal_call=False):
        rospy.loginfo('module type : List')

        if bb is None:
            bb = self.bbox

        object_name = request.target_name
        sort_option = request.sort_option

        response_list = RecognitionListResponse()
        coordinate_list = []
        bbox_list = self.createBboxList(bb)

        # 座標を格納したlistを作成
        for i in range(len(bbox_list)):
            if object_name == 'any':
                if not(bbox_list[i] in self.object_dict['any']): continue
            elif object_name != '':
                if not(bbox_list[i] == object_name): continue
            coordinate_list.append([bbox_list[i], [int((bb[i].ymin + bb[i].ymax)/2), int((bb[i].xmin + bb[i].xmax)/2)]])

        # ソート
        if sort_option == 'left':
            coordinate_list.sort(key=lambda x: x[1][1])
        elif sort_option == 'center':
            for i in coordinate_list:
                i[1][1] -= (self.image_width)/2
            coordinate_list.sort(key=lambda x: abs(x[1][1]))
            for i in coordinate_list:
                i[1][1] += (self.image_width)/2
        elif sort_option == 'right':
            coordinate_list.sort(key=lambda x: x[1][1], reverse=True)

        # 内部呼び出しかserverの呼び出しか
        if internal_call:
            response_list.object_list = coordinate_list
        else:
            for i in coordinate_list:
                response_list.object_list.append(i[0])

        return response_list

    def countObject(self, object_name='', bb=None):
        rospy.loginfo('module type : Count')

        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target_name

        object_count = 0
        bbox_list = self.createBboxList(bb)

        if object_name == 'any':
            for i in range(len(bbox_list)):
                if bbox_list[i] in self.object_dict['any']:
                    object_count += 1
        else:
            object_count = bbox_list.count(object_name)
        return object_count

    def findObject(self, object_name=''):
        rospy.loginfo('module type : Find')
        mimi_control = MimiControl()

        if type(object_name) != str:
            object_name = object_name.target_name

        find_flg = self.countObject(object_name)
        loop_count = 0

        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1

            rotation_angle = 45 - (((loop_count)%4)/2) * 90
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(3.0)

            bbox_list = self.createBboxList(self.bbox)
            if object_name == '':
                find_flg = bool(len(bbox_list))
            elif object_name == 'any':
                find_flg = bool(len(list(set(self.object_dict['any'])&set(bbox_list))))
            else:
                find_flg = object_name in bbox_list
        return find_flg

    def localizeObject(self, object_name='', sort_request=[], bb=None):
        sort
        rospy.loginfo('module type : Localize')
        Detector = CallDetector()

        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target_name

        object_centroid = Point()
        object_centroid.x = numpy.nan
        object_centroid.y = numpy.nan
        object_centroid.z = numpy.nan

        bbox_list = self.createBboxList(bb)
        object_count = self.countObject(object_name)

        if object_name == 'any':
            exist_flg = bool(object_count)
            if exist_flg:
                object_list = self.listObject(object_name)
                object_name = object_list[0]
        else:
            exist_flg = bool(object_count)

        if not exist_flg:
            return object_centroid

        index_num = bbox_list.index(object_name)
        center_x = int((bb[index_num].ymin + bb[index_num].ymax)/2)
        center_y = int((bb[index_num].xmin + bb[index_num].xmax)/2)
        rospy.sleep(0.5)
        Detector.detectorService(center_x, center_y)
        object_centroid = Detector.object_centroid
        return object_centroid


if __name__ == '__main__':
    rospy.init_node('recognition_tools')
    recognition_tools = RecognitionTools()
    rospy.spin()
