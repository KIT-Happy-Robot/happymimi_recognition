#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rospy
import roslib
import tf2_ros
import rosparam
import actionlib
from geometry_msgs.msg import Point
from happymimi_msgs.srv import SimpleTrg, SimpleTrgResponse
from happymimi_recognition_msgs.srv import MultipleLocalize
from happymimi_recognition_msgs.msg import PubObjectTFAction, PubObjectTFGoal


file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl


class GenerateObjectCoord():
    def __init__(self):
        self.sac = actionlib.SimpleActionClient('pub_object_tf', PubObjectTFAction)
        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Value
        self.sac.wait_for_server()
        self.goal = PubObjectTFGoal()
        self.rate = rospy.Rate(10.0)
        self.object_dict = {}

    def execute(self, frame_name, dist_x, dist_y):
        self.goal.name = frame_name
        self.goal.dist_x = dist_x
        self.goal.dist_y = dist_y
        self.sac.send_goal(self.goal)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('map', frame_name, rospy.Time())
                self.object_dict[frame_name] = []
                self.object_dict[frame_name].append(trans.transform.translation.x)
                self.object_dict[frame_name].append(trans.transform.translation.y)
                self.object_dict[frame_name].append(trans.transform.rotation.z)
                self.object_dict[frame_name].append(trans.transform.rotation.w)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            self.rate.sleep()
        self.sac.cancel_goal()
        print(f"ghc >>> {self.object_dict}")
        return self.object_dict


class ObjectCoordGeneratorSrv():
    def __init__(self):
        hcg_srv = rospy.Service('object_coord_generator', SimpleTrg, self.execute)
        rospy.loginfo("Ready to object_coord_generator server")
        # Service
        self.ml_srv = rospy.ServiceProxy('/recognition/multiple_localize', MultipleLocalize)
        # Param
        self.map_range = rospy.get_param('/map_range')
        # Value
        self.dist_data = MultipleLocalize()
        self.ghc = GenerateObjectCoord()
        self.bc = BaseControl()
        self.object_coord_dict = {}
        self.h_dict_count = 0
        self.i = 0

    def saveDict(self):
        param_path = roslib.packages.get_pkg_dir("happymimi_params")
        #os.remove(param_path + '/location/'  + 'tmp_object_location.yaml')
        print(f"set_param >>> {self.object_coord_dict}")
        rospy.set_param('/tmp_object_location', self.object_coord_dict)
        rosparam.dump_params(param_path + '/location/'  + 'tmp_object_location.yaml', '/tmp_object_location')

    def judgeMapin(self, coord):
        self.map_range = rospy.get_param('/map_range')
        rpy = coord
        print(f"coord --->>> {coord}")
        # print rpy
        if coord[0] < self.map_range["min_x"] or coord[0] > self.map_range["max_x"]:
            jm_result = False
        elif coord[1] < self.map_range["min_y"] or coord[1] > self.map_range["max_y"]:
            jm_result = False
        else:
            jm_result = True
        return jm_result

    # def change_dict_key(self, d, old_key, new_key):
    def change_dict_key(self, d, old_key):
        #self.object_coord_dict.clear()
        new_key = "object_" + str(len(self.object_coord_dict) + 1)
        #new_key = "object_" + str(self.i)
        #new_key = "object_0"
        print(new_key)
        d[new_key] = d[old_key]
        print(old_key)
        del d[old_key]

    def createDict(self, list_len):
        # map座標系に変換してlocation dictを作成
        print(list_len)
        for i in range(list_len):
            frame_id = "object_" + str(i)
            #frame_id + "object_0"
            object_dict = self.ghc.execute(frame_id, self.dist_data.points[i].x, self.dist_data.points[i].y)
            #print(f"object dist -->> {object_dict}")
            print(frame_id)
            if self.judgeMapin(object_dict[frame_id]):
                print("judgeMapin = True")
                #new_id = "object_" + str(self.h_dict_count)
                if frame_id in self.object_coord_dict:
                    # self.change_dict_key(object_dict, frame_id, new_id)
                    print("execute change_dict_key")
                    self.change_dict_key(object_dict, frame_id)
                self.object_coord_dict.update(object_dict)
                print (f"self.object_coord_dict{self.object_coord_dict} in createDict")
                self.h_dict_count += 1
            else:
                print("judgeMapin = False")
                pass
        #self.h_dict_count += 1
       # self.i += 1

    def execute(self, srv_req):
        # while len(self.object_coord_dict) < 1:
        # for i in range(2):
        #for i in range(1):
        print ("count num: " + str(self.h_dict_count))
        # if i != 0:
            # self.bc.rotateAngle(-45, 0.3)
        # 人がいるか
        self.dist_data = self.ml_srv(target_name = "person")
        print(self.dist_data)
        list_len  = len(list(self.dist_data.points))
        # print list_len
        if list_len == 0 :
            # self.bc.rotateAngle(-75)
            # rospy.sleep(2.0)
            pass
        else:
            self.createDict(list_len)
            # 台車の回転
            #if i < 2:
            #    self.bc.rotateAngle(-50, 0.3)
            #    rospy.sleep(1.0)
       # self.object_coord_dict.clear()
        self.saveDict()
        print("====================")
        print(self.object_coord_dict)
        self.object_coord_dict.clear()
        return SimpleTrgResponse(result = True)


if __name__ == '__main__':
    rospy.init_node('object_coord_generator')
    try:
        hcgs = ObjectCoordGeneratorSrv()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
