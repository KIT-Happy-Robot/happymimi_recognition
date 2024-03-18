#!/usr/bin/env python3

# 点群で目の前のテーブル上にある物体の3次元位置、距離、形状、傾きなどをROSサービス通信で返すサービスサーバー

import os
import yaml
import numpy as np
from pathlib import Path
import cv2
import open3d as o3d

import rospy
import cv_bridge
from happymimi_recognition_msgs import UOR, UORResponse
from pc_module import PointCloudModule


# IN: Points | OUT: UOR(centroid 3d coords, ,size, shape)
class TableObjectServer():
    def __init__(self):
        rospy.init_node("table_object_server")
        rospy.loginfo("Initializing Node: ** table_object_server **")
        self.PCM = PointCloudModule()
        self.PCM.rosInit()
        self.to_ss = rospy.Service("/uor/table_object_server", UOR, self.serviceCB)
        rospy.loginfo("TableObjectServer: Im Ready to response...")
        
    def serviceCB(self, req):
        
        
    def showO3dPoints(self, pcd):
        plane_pcd = o3d.io.read_point_cloud("./pcd_plane.ply")
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(plane_pcd)
        vis.run()
    
    def debugPlane(self, ):
        
        plane_pcd = o3d.io.read_point_cloud("./pcd_plane.ply")
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(plane_pcd)
        vis.run()
    def showPcd(self):
        self.showO3dPoints(self.o3d_color_pcd)    