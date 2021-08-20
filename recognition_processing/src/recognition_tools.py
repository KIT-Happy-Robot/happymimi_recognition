#!/usr/bin/env python
# -*- coding: utf-8 -*

import time
import numpy
import rospy
import rosparam
from geometry_msgs.msg import Twist, Point
from darknet_ros_msgs.msg import BoundingBoxes
# -- Custom Message --
from happymimi_recognition_msgs.srv import RecognizeFind, RecognizeCount, RecognizeLocalize, PositionEstimator

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
        rospy.Service('/recognition/find',RecognizeFind,self.findObject)
        rospy.Service('/recognition/count',RecognizeCount,self.countObject)
        rospy.Service('/recognition/localize',RecognizeLocalize,self.localizeObject)

        self.object_dict = rosparam.get_param('/object_dict')
        self.bbox = []
        self.update_time = 0 # darknetからpublishされた時刻を記録
        self.update_flg = False # darknetからpublishされたかどうかの確認

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb.bounding_boxes

    def createBboxList(self,bb):
        bbox_list = []
        for i in range(len(bb)):
            bbox_list.append(bb[i].Class)
        return bbox_list

    def initializeBBox(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            if time.time() - self.update_time > 1.5 and self.update_flg:
                self.bbox = []
                self.update_flg = False
                rospy.loginfo('initialize')
            rate.sleep()

    def findObject(self, object_name='None'):
        rospy.loginfo('module type : Find')
        mimi_control = MimiControl()
        
        if type(object_name) != str:
            object_name = object_name.target_name
            
        find_flg, _ = self.countObject(object_name)
        loop_count = 0
        
        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1
            
            rotation_angle = 45 - (((loop_count)%4)/2) * 90
            mimi_control.angleRotation(rotation_angle)
            rospy.sleep(3.0)

            bbox_list = self.createBboxList(self.bbox)
            if object_name == 'None':
                find_flg = bool(len(bbox_list))
            elif object_name == 'any':
                find_flg = bool(len(list(set(self.object_dict['any'])&set(bbox_list))))
            else:
                find_flg = object_name in bbox_list
        return find_flg
            
    def countObject(self, object_name='None', bb=None):
        rospy.loginfo('module type : Count')
        
        if bb is None:
            bb = self.bbox
        if type(object_name) != str:
            object_name = object_name.target_name
            
        object_list = []
        bbox_list = self.createBboxList(bb)
        
        if object_name == 'any':
            any_dict = {}
            for i in range(len(bbox_list)):
                if bbox_list[i] in self.object_dict['any']:
                    any_dict[bbox_list[i]] = bb[i].xmin
            sorted_any_dict = sorted(any_dict.items(), key=lambda x:x[1])
            for i in range(len(sorted_any_dict)):
                object_list.append(sorted_any_dict[i][0])
            object_count = len(sorted_any_dict)
        else:
            object_count = bbox_list.count(object_name)
            object_list = bbox_list
        return object_count, object_list

    def localizeObject(self, object_name='None', bb=None):
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
        object_count, object_list = self.countObject(object_name)

        if object_name == 'any':
            exist_flg = bool(len(object_list))
            if exist_flg: object_name = object_list[0]
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
