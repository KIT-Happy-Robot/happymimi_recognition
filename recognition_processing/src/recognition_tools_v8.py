#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import rosparam
import rosgraph
import roslib.packages
import os
import sys
import time
import numpy
import yaml
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from ultralytics import YOLO
from ultralytics_ros.msg import YoloResult
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from happymimi_msgs.srv import StrTrg
from happymimi_recognition_msgs.srv import (RecognitionList, RecognitionListRequest, RecognitionListResponse,
                                            RecognitionCount, RecognitionCountRequest, RecognitionCountResponse,
                                            RecognitionFind, RecognitionFindRequest, RecognitionFindResponse,
                                            RecognitionLocalize, RecognitionLocalizeRequest, RecognitionLocalizeResponse,
                                            MultipleLocalize, MultipleLocalizeRequest, MultipleLocalizeResponse,
                                            PositionEstimator, PositionEstimatorRequest)


teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl

param_path = roslib.packages.get_pkg_dir('recognition_processing') + "/param/"
yaml_file_path = param_path + "yolo_v8_object_id.yaml"

class CallDetector(object):
    def __init__(self):
        self.detect_depth = rospy.ServiceProxy('/detect/depth', PositionEstimator)

        self.object_centroid = Point()

    def detectorService(self, center_x, center_y):
        rospy.wait_for_service('/detect/depth')
        position_estimator_req = PositionEstimatorRequest()
        position_estimator_req.center_x = int(center_x)
        position_estimator_req.center_y = int(center_y)
        print(position_estimator_req.center_x, position_estimator_req.center_y)
        res = self.detect_depth(position_estimator_req)
        self.object_centroid = res.point


class RecognitionToolsV8(object):
    bbox = []

    def __init__(self):
        rospy.Subscriber('/yolo_result',YoloResult,self.boundingBoxCB)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        rospy.Service('/recognition/save',StrTrg,self.saveImage)
        rospy.Service('/recognition/list',RecognitionList,self.listObject)
        rospy.Service('/recognition/find',RecognitionFind,self.findObject)
        rospy.Service('/recognition/count',RecognitionCount,self.countObject)
        rospy.Service('/recognition/localize',RecognitionLocalize,self.localizeObject)
        rospy.Service('/recognition/multiple_localize',MultipleLocalize,self.multipleLocalize)

        self.realsense_image = Image()
        self.image_height = 480# rosparam.get_param('/camera/realsense2_camera/color_height')
        self.image_width = 640# rosparam.get_param('/camera/realsense2_camera/color_width')
        
        #rosparam空間にidの辞書を登録
        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
        for key, value in yaml_data.items():
            rospy.set_param("/yolo_v8_object_id", value)
        self.object_id = rosparam.get_param('/yolo_v8_object_id')
        
        try:
            self.object_dict = rosparam.get_param('/object_dict')
            
        except rosgraph.masterapi.MasterError:
            self.object_dict = {'any':['cup', 'bottle']}

        self.update_time = 0 # darknetからpublishされた時刻を記録
        self.update_flg = False # darknetからpublishされたかどうかの確認

        rospy.Timer(rospy.Duration(0.5), self.initializeBbox)

    def boundingBoxCB(self,bb):
        self.update_time = time.time()
        self.update_flg = True
        RecognitionToolsV8.bbox = bb

    def initializeBbox(self, event):
        # darknetが何も認識していない時にRecognitionTools.bboxを初期化する
        if time.time() - self.update_time > 1.0 and self.update_flg:
            RecognitionToolsV8.bbox = []
            self.update_flg = False
            rospy.loginfo('initialize')

    def createBboxList(self,bb):
        bbox_list = []
        for i in bb.detections.detections:
            for j in range(len(i.results)):
                obj = i.results[j].id
                obj_name = self.object_id[str(obj)]
                bbox_list.append(obj_name)
        return bbox_list

    def realsenseCB(self, image):
        self.realsense_image = image

    def saveImage(self, req, bb=None):
        if bb is None:
            bb = RecognitionToolsV8.bbox
        bbox_list = self.createBboxList(bb)

        bridge = CvBridge()
        cv2_image = bridge.imgmsg_to_cv2(self.realsense_image, desired_encoding="bgr8")

        font = cv2.FONT_HERSHEY_SIMPLEX
        for i, name in enumerate(bbox_list):
            cv2.rectangle(cv2_image,(bb[i].xmin,bb[i].ymin),(bb[i].xmax,bb[i].ymax),(0,255,0),2)
            pix_y = bb[i].ymin-5
            if pix_y<10: pix_y=10
            cv2.putText(cv2_image, name, (bb[i].xmin,pix_y),font,0.5,(0,0,0))
        cv2.imwrite(req.data+"/"+str(time.time())+".png",cv2_image)
        return True

    def listObject(self, request, bb=None, internal_call=False):
        rospy.loginfo('module type : List')

        response_list = RecognitionListResponse()
        coordinate_list = []

        object_name = request.target_name
        sort_option = request.sort_option
        if bb is None:
            bb = RecognitionToolsV8.bbox
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
        elif sort_option == 'front':
            name_list = set([row[0] for row in coordinate_list])

            localize_req = RecognitionLocalizeRequest()
            localize_req.sort_option.data = 'left'
            depth_list = []

            for name in name_list:
                loop_count = self.countObject(RecognitionCountRequest(name), bb=bb).num
                localize_req.target_name = name
                for i in range(loop_count):
                    localize_req.sort_option.num = i
                    centroid = self.localizeObject(localize_req, bb=bb).point
                    depth_list.append([name, centroid])
            depth_list.sort(key=lambda x: x[1].x)

        try:
            response_list.object_list = depth_list
        except NameError:
            response_list.object_list = coordinate_list

        # serverの呼び出し
        if not internal_call:
            response_list.object_list = [row[0] for row in response_list.object_list]
        return response_list

    def countObject(self, request, bb=None):
        rospy.loginfo('module type : Count')

        response_count = RecognitionCountResponse()
        object_count = 0

        object_name = request.target_name
        if bb is None:
            bb = RecognitionToolsV8.bbox
        bbox_list = self.createBboxList(bb)

        if object_name == 'any':
            for i in range(len(bbox_list)):
                if bbox_list[i] in self.object_dict['any']:
                    object_count += 1
        else:
            object_count = bbox_list.count(object_name)
        response_count.num = object_count
        return response_count

    def findObject(self, request):
        rospy.loginfo('module type : Find')

        base_control = BaseControl()

        response_flg = RecognitionFindResponse()
        object_name = request.target_name
        loop_count = 0

        find_flg = bool(self.countObject(RecognitionCountRequest(object_name)).num)

        while not find_flg and loop_count <= 3 and not rospy.is_shutdown():
            loop_count += 1

            rotation_angle = 30 - (((loop_count)%4)/2) * 60
            base_control.rotateAngle(rotation_angle, 0.5)
            rospy.sleep(3.0)

            bbox_list = self.createBboxList(RecognitionToolsV8.bbox)
            if object_name == '':
                find_flg = bool(len(bbox_list))
            elif object_name == 'any':
                find_flg = bool(len(list(set(self.object_dict['any'])&set(bbox_list))))
            else:
                find_flg = object_name in bbox_list
        response_flg.result = find_flg
        return response_flg

    def localizeObject(self, request, bb=None):
        rospy.loginfo('module type : Localize')

        Detector = CallDetector()

        response_centroid = RecognitionLocalizeResponse()
        response_centroid.point.x = numpy.nan
        response_centroid.point.y = numpy.nan
        response_centroid.point.z = numpy.nan

        object_name = request.target_name
        sort_option = request.sort_option
        if bb is None:
            bb = RecognitionToolsV8.bbox
        bbox_list = self.createBboxList(bb)

        exist_flg = bool(self.countObject(RecognitionCountRequest(object_name), bb=bb).num)

        # 対象の物体が存在しない場合
        if not exist_flg:
            return response_centroid

        # リストの取得
        list_req = RecognitionListRequest()
        list_req.target_name = object_name
        list_req.sort_option = sort_option.data
        object_list = self.listObject(request=list_req, bb=RecognitionToolsV8.bbox, internal_call=True).object_list
        try:
            center_x, center_y = object_list[sort_option.num][1]
        except IndexError:
            return response_centroid

        # 三次元位置の推定
        rospy.sleep(0.5)
        Detector.detectorService(center_x, center_y)
        response_centroid.point = Detector.object_centroid
        return response_centroid

    def multipleLocalize(self, request, bb=None):
        rospy.loginfo('module type : AddvancedLocalize')

        response_centroid = MultipleLocalizeResponse()

        object_name = request.target_name
        if bb is None:
            bb = RecognitionToolsV8.bbox
        bbox_list = self.createBboxList(bb)

        # リストの取得
        list_req = RecognitionListRequest()
        list_req.target_name = object_name
        list_req.sort_option = 'front'
        object_list = self.listObject(request=list_req, bb=RecognitionToolsV8.bbox, internal_call=True).object_list
        
        #response_centroid.points = [row[1] for row in object_list if not(row[1].x is numpy.nan)]
        response_list = []
        for row in object_list:
            if not(row[1].x is numpy.nan) and row[1].x > 0.1:
                response_list.append(row[1])
        response_centroid.points = response_list
        return response_centroid


if __name__ == '__main__':
    rospy.init_node('recognition_tools_v8')
    recognition_tools_v8 = RecognitionToolsV8()
    rospy.spin()
