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
import sensor_msgs.point_cloud2 as pc2
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
            if not color: rospy.Subscriber('/camera/depth/points', Image, self.headPointsCB, queue_size=1)
            else: rospy.Subscriber('/camera/depth/color/points', Image, self.headColorPointsCB, queue_size=1)
        if arm:
            if not color: rospy.Subscriber('/camera/depth/points_arm', Image, self.armPointsCB, queue_size=1) ###
            else: rospy.Subscriber('/camera/depth/color/points_arm', Image, self.armColorPointsCB, queue_size=1) ###

    def headPointsCB(self, msg):
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(msg.xyz)
        #o3d.visualization.draw_geometries([self.o3d_pcd]) # 可視化
    def headColorPointsCB(self, msg): # OUT: self.o3d_color_pcd.points of head
        self.head_color_points = pc2.read_points(msg, skip_nans=True) # 各点の座標を取得&それをNumpyArrayの形式で返
        self.head_color_points_array = []
        for p in self.head_color_points_points:
            self.head_color_points_array.append([p[0], p[1], p[2]])
        self.o3d_color_pcd = o3d.geometry.PointCloud()
        self.o3d_color_pcd.points = o3d.utility.Vector3dVector(msg.xyz) #座標データをOpen3DのPointCloudに追加
    #def armPointsCB(self, msg):
    #def armColorPointsCB(self, msg):
    def depthImageCB(self, msg): # OUT: self.depth_image
        try: self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e: rospy.logerr(e)
    # 要見直し。IN:pcd, options -------
    def getWidthSliceDepth(self, area=None): ###
        height, width = self.depth_image.shape[:2] # # 画像の幅と高さを取得
        left = int(width / 4) # 左右方向の範囲を定義（画面横幅の1/4）
        right = int(width * 3 / 4)
        depth_slice = self.depth_image[:, left:right] # 指定された範囲の深度データを抽出
        depth_slice_valid = depth_slice[depth_slice > 0] # 深度が0（無効値）のピクセルを除外
    #def getHeightSlice(self, area): pass
    #def getSlice(self, area): pass
    def getNearestDistance(self, di):
        if len(id) > 0:
            min_depth = np.min(id) # 最も近い深度を取得
            rospy.loginfo("\n最も浅い深度: {:.2f} メートル".format(min_depth))
        else:
            rospy.loginfo("\ngetNearestDistance: 指定された範囲内に深度データがありません")
        return min_depth # + 0.05
    # ----------------------------------
    # Open3d (process head color points)------------------------
    # downsampling
    def getHeadDownsampledColorPoints(self, pcd, voxel_size=0.01):
        ds_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        return ds_pcd
    def getDistanceRemevedPoints(self, pcd, threshold_distance):
        distances = pcd.compute_point_cloud_distance(pcd) # Compute distance of each point from the origin
        indices = distances < threshold_distance
        return pcd.select_by_index(indices)
    # Extraction（invert平面かそれ以外の点群かのやつ）
    def getPlanePoints(self, pcd, invert=False, distance_threshold=0.01, ransac_n=3, num_iterations=1000): # 
        # distance_threshold：インライアとするしきい値, ransac_n：平面を推定する最小の点数, num_iterations：試行回数
        plane_model, inliers = pcd.segment_plane(distance_threshold, 
                                                            ransac_n, 
                                                            num_iterations)
        [a, b, c, d] = plane_model # each 3d point
        # [a, b, c, d] = plane_model.np.tolist()
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        # DEBUG
        inlier_indices = inliers.indices # 平面に属する点のインデックスを取得
        inlier_cloud = pcd.select_by_index(inliers, invert=invert)
        return inlier_cloud # 平面点のみを抽
        #inlier_cloud.paint_uniform_color([1.0, 0, 0])
        
    def getPointOnPlane(self, pcd): # 平面上の点群を抽出
        plane_pcd = self.getPlanePoints(pcd)
        return plane_pcd
    
    def extractPointsOnTable(self): pass
    def extractTableObjects(self):###
        centroid_num = 0
        centroid_coords = []
        size = ["w","h"]; shape=""
        
        return centroid_num, centroid_coords