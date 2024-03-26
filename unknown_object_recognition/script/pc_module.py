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
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge


class PointCloudModule():
    def __init__(self):
        print("Initializing PointCloudHub object...")
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
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
        self.color_points = msg
        self.num_color_pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        # self.color_points = pc2.read_points(msg, skip_nans=True) # 各点のXYZ座標のみを抽出&それをNumpyArrayの形式で返
        # self.color_points_array = []
        # for p in self.color_points:
        #     self.color_points_array.append([p[0], p[1], p[2]])
        # self.o3d_color_pcd = o3d.geometry.PointCloud()
        # self.o3d_color_pcd.points = o3d.utility.Vector3dVector(msg.xyz) #座標データをOpen3DのPointCloudに追加
    def getHeadO3dColorPoints(self):
        # Convert numpy array to Open3D point cloud
        self.head_o3d_color_pc = o3d.geometry.PointCloud()
        self.head_o3d_color_pc.points = o3d.utility.Vector3dVector(self.num_color_pc)
        ds_pcd = self.head_o3d_color_pc.voxel_down_sample(voxel_size=0.01)
        # Visualize the downsampled point cloud
        self.vis.clear_geometries()
        self.vis.add_geometry(ds_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        return ds_pcd
        
    def armPointsCB(self, msg):
        self.arm_o3d_pcd = o3d.geometry.PointCloud()
        self.arm_o3d_pcd.points = o3d.utility.Vector3dVector(msg.xyz)
    def armColorPointsCB(self, msg): pass
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
        if len(di) > 0:
            min_depth = np.min(di) # 最も近い深度を取得
            rospy.loginfo("\n最も浅い深度: {:.2f} メートル".format(min_depth))
        else:
            rospy.loginfo("\ngetNearestDistance: 指定された範囲内に深度データがありません")
        return min_depth # + 0.05

    # Open3Dで処理した点群をROSのPointCloud2メッセージに変換する関数
    def convertO3dPc2(self, o3d_pcd):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link"  # フレームIDを適切なものに変更する必要があります
        # ROSのPointCloud2メッセージに変換
        pc2_msg = ros_numpy.point_cloud2.array_to_pointcloud2(o3d_pcd.points, o3d_pcd.colors, header)
        return pc2_msg
    # DEBUG ----------------------------------------------------------------
    #def publishO3dPcd(self, o3d_pcd):
    # Open3Dで処理した点群の表示
    def visualize_open3d_pointcloud(o3d_pcd):
        o3d.visualization.draw_geometries([o3d_pcd])

    # Open3d (process head color points)###
    # 前処理-------------------------------
    # downsampling
    def downsamplePoints(self, pcd, voxel_size=0.01): return pcd.voxel_down_sample(voxel_size=voxel_size)
    # 距離除去
    def getRemevedOutlierPoints(self, pcd, distance_threshold=1.2):
        # ソース点群からターゲット点群までの距離を計算
        distances = pcd.compute_point_cloud_distance([0,0,0]) # Compute distance of each point from the origin
        indices = distances < distance_threshold
        return pcd.select_by_index(indices)
    
    # RANSACアルゴリズムの平面抽出（invert:平面かそれ以外の点群かのやつ）
    def getPlanePoints(self, pcd, invert=False, distance_threshold=0.01, ransac_n=3, num_iterations=1000): # 
        plane_model, inliers = pcd.segment_plane(distance_threshold, # distance_threshold：インライアとするしきい値
                                                            ransac_n, # ransac_n：平面を推定する最小の点数
                                                            num_iterations) # num_iterations：試行回数
        [a, b, c, d] = plane_model # each 3d point
        # [a, b, c, d] = plane_model.np.tolist()
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        inlier_indices = inliers.indices # 平面に属する点のインデックスを取得
        inlier_cloud = pcd.select_by_index(inliers, invert=invert) # T:平面点群以外、F:平面点群
        return inlier_cloud # 平面点のみを抽
        #inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # テーブルの上にある物体の点群だけ抽出
    def extractPointsOnTable(self, pcd, distance_threshold=1.2):
        head_ds_points = self.downsamplePoints(pcd, distance_threshold)
        inliers = self.getRemevedOutlierPoints(head_ds_points)
        objects_points = self.getPlanePoints(inliers, invert=False)
        return objects_points
    # ---------------------------------------------
    # セグメント化（クラスタリングアルゴリズム（DBSCANやK-meansなど）
    def clastering(self, pcd):
        # 平面上の物体の点群をクラスタリング（Density-Based Spatial Clustering of Applications with Noise）
        # デバッグ用に処理の詳細な情報をコンソールに表示
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            # 各点がどのクラスタに属するかを示すインデックス（numpy.ndarray）、-1はノイズ
            labels = np.array(pcd.cluster_dbscan(eps=0.02, # クラスタ内の点と定義する距離の閾値
                                                min_points=10, # クラスタを形成するために必要な最小の点の数
                                                print_progress=True)) # クラスタリングの進行状況を表示するかどうか
        max_label = labels.max()
        print(f"Point cloud has {max_label + 1} clusters")
        # クラスタリングされた点群を取得
        clustered_point_clouds = []
        for label in range(max_label + 1):
            cluster_indices = np.where(labels == label)[0]
            cluster_point_cloud = pcd.select_by_index(cluster_indices)
            clustered_point_clouds.append(cluster_point_cloud)
        return labels, clustered_point_clouds
    # 各クラスタの点群を取得する関数
    # def getClusterPointClouds(self, pcd, labels):
    #     cluster_point_clouds = []
    #     for label in np.unique(labels):
    #         if label == -1:
    #             continue  # ノイズはスキップ
    #         cluster_indices = np.where(labels == label)[0]
    #         cluster_point_cloud = pcd.select_by_index(cluster_indices)
    #         cluster_point_clouds.append(cluster_point_cloud)
    #     return cluster_point_clouds
    # 各クラスタの重心を計算する関数
    def computeClusterCentroids(self, cluster_point_clouds):
        centroids = []
        for cluster_point_cloud in cluster_point_clouds:
            centroid = np.asarray(cluster_point_cloud.points).mean(axis=0)
            centroids.append(centroid)
        return centroids

    # カメラ原点から各重心までの距離を計算する関数
    def computeDistancesToCentroids(self, centroids, base_coord): ###
        camera_origin = np.array([0, 0, 0])  # カメラ原点の座標
        distances = []
        for centroid in centroids:
            distance = np.linalg.norm(centroid - camera_origin)
            distances.append(distance)
        # 距離が小さい順に並び替える
        sorted_indices = sorted(range(len(distances)), key=lambda k: distances[k])
        
        # クラスタ点群の3次元座標のリストを距離が小さい順に並べ替える
        sorted_centroids = [centroids[i] for i in sorted_indices]
        sorted_distances = [distances[i] for i in sorted_indices]
        return sorted_centroids, sorted_distances
        #return distances
    
    # テーブルの上においてある物体の各情報を求める
    def extractTableObjects(self, pcd):###
        points_on_table = self.extractPointsOnTable(pcd)
        labels, clustered_point_clouds = self.clastering(points_on_table)
        #clusterd_point_clouds = self.getClusterPointClouds(clustered_points, labels)
        each_claster_centroids = self.computeClusterCentroids(clustered_point_clouds)
        centroid_coords, centroid_distances = self.computeDistancesToCentroids(each_claster_centroids)
        # 距離が小さい順に並び替える
        #sorted_distances = sorted(centroids_distances)
        min_distance = centroid_distances[0]
        centroid_num = len(centroid_distances)
        nearest_centroid_coord = each_claster_centroids[0]
        size = ["w","h"]; shape=""; tilt_angle=0.0
        return centroid_num, centroid_coords

    
    # 3DoF推定 --------------------------------------
    def estimateOrientation(self, claster_color_pcd):
        # Open3DのPointCloudオブジェクトに直接設定
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(claster_color_pcd[:, :3])  # 座標情報を設定
        pcd.colors = o3d.utility.Vector3dVector(pcd[:, 3:6] / 255.0)  # 色情報を設定（0から1の範囲に正規化）
        centroid = np.mean(np.asarray(pcd.points), axis=0) # 重心計算（各次元の平均値を求める）
        pcd.translate(-centroid) # 中心化（各点の座標から重心を引く）
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # 法線ベクトルを計算
        pca = o3d.geometry.PointCloudPCA(pcd) # PCA主成分分析で物体向き推定
        orientation_vector = pca.get_eigenvectors()[0]  # 最大の固有ベクトルを物体の向きとする
        return orientation_vector
