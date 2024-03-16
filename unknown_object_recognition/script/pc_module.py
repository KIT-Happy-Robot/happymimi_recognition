#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import math
import numpy as np

import cv2
import torch
import open3d as o3d
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge


class PointCloudModule():
    def __init__(self):
        print("Initializing PointCloudHub object...")
        self.point =[]
        self.points = []
        self.pc2 = []

    # できれば1ノードで1Subscriberまで
    def rosInit(self, head=True, arm=False, color=False):
        rospy.loginfo(f"\nPointCloudModule: Initializing ROS: head:{head}, arm:{arm}")
        if head:
            if not color: rospy.Subscriber('/camera/depth/points', Image, self.setHeadPointsCB, queue_size=1)
            else: rospy.Subscriber('/camera/depth/color/points', Image, self.setHeadColorPointsCB, queue_size=1)
        if arm:
            if not color: rospy.Subscriber('/camera/depth/points_arm', Image, self.setArmPointsCB, queue_size=1) ###
            else: rospy.Subscriber('/camera/depth/color/points_arm', Image, self.setArmColorPointsCB, queue_size=1) ###

        
    def setHeadPointsCB(self, msg):
        #pcl = pointcloud2_to_xyz_array(pcl_msg) ###
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(msg.xyz)
        #o3d.visualization.draw_geometries([self.o3d_pcd]) # ポイントクラウドの可視化など
    def setHeadColorPointsCV(self, msg):
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(msg.xyz)
    def getO3dPoints(self): return self.o3d_pcd
    def setDepthImageCB(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(e)
    def getWidthSlice(self, area): ###
        height, width = self.depth_image.shape[:2] # # 画像の幅と高さを取得
        left = int(width / 4) # 左右方向の範囲を定義（画面横幅の1/4）
        right = int(width * 3 / 4)
        depth_slice = self.depth_image[:, left:right] # 指定された範囲の深度データを抽出
        depth_slice_valid = depth_slice[depth_slice > 0] # 深度が0（無効値）のピクセルを除外
    def getHeightSlice(self, area): pass ###
    def getSlice(self, area): pass ###
    def getNearestDistance(self, di):
        if len(id) > 0:
            min_depth = np.min(id) # 最も近い深度を取得
            rospy.loginfo("\n最も浅い深度: {:.2f} メートル".format(min_depth))
        else:
            rospy.loginfo("\ngetNearestDistance: 指定された範囲内に深度データがありません")
        return min_depth # + 0.05
    # Open3d ------------------------
    # downsample
    # 
    # Extraction
    def getPlanePoints(self, option=None, distance_threshold=0.01, ransac_n=3, num_iterations=1000): # 
        # distance_threshold：インライアとするしきい値, ransac_n：平面を推定する最小の点数, num_iterations：試行回数
        plane_model, inliers = self.o3d_pcd.segment_plane(distance_threshold, 
                                                            ransac_n, 
                                                            num_iterations)
        [a, b, c, d] = plane_model # each 3d point
        # [a, b, c, d] = plane_model.np.tolist()
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        # DEBUG
        inlier_cloud = self.o3d_pcd.select_by_index(inliers) # インライアの点を抽出して色を付け
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        if option: return self.o3d_pcd.select_by_index(inliers, invert=False)
        if not option: return self.o3d_pcd.select_by_index(inliers, invert=True)
        if option is None: return self.o3d_pcd.select_by_index(inliers, invert=False)
    def getPointOnPlane(self): # 平面上の点群を抽出
        plane_pcd = self.getPlanePoints()
        return 
    
    def extractPointsOnTable(self): pass
    def extractTableObjects(self):###
        centroid_num = 0
        centroid_coords = []
        return centroid_num, centroid_coords