#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
import cv2
import torch
import open3d as o3d
import rospy
import ros_numpy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2


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
            rospy.Subscriber('/camera/depth/points', PointCloud2, self.headPointsCB, queue_size=1)
            if color:
                print("\n Subscribe: '/camera/depth/color/points', PointCloud2, self.headColorPointsCB, queue_size=1")
                rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.headColorPointsCB, queue_size=1)
    
    def headPointsCB(self, msg):
        self.o3d_pcd = o3d.geometry.PointCloud()
        self.o3d_pcd.points = o3d.utility.Vector3dVector(msg.xyz)
        #o3d.visualization.draw_geometries([self.o3d_pcd]) # 可視化
    def headColorPointsCB(self, msg): # OUT: self.o3d_color_pcd.points of head
        self.color_points = msg
        
        # self.color_points = pc2.read_points(msg, skip_nans=True) # 各点のXYZ座標のみを抽出&それをNumpyArrayの形式で返
        # self.color_points_array = []
        # for p in self.color_points:
        #     self.color_points_array.append([p[0], p[1], p[2]])
        # self.o3d_color_pcd = o3d.geometry.PointCloud()
        # self.o3d_color_pcd.points = o3d.utility.Vector3dVector(msg.xyz) #座標データをOpen3DのPointCloudに追加
    def getHeadO3dColorPoints(self):
        self.num_color_pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.color_points)
        # Convert numpy array to Open3D point cloud
        self.head_o3d_color_pc = o3d.geometry.PointCloud()
        self.head_o3d_color_pc.points = o3d.utility.Vector3dVector(self.num_color_pc)
        ds_pcd = self.head_o3d_color_pc.voxel_down_sample(voxel_size=0.01)
        # Visualize the downsampled point cloud
        # self.vis.clear_geometries()
        # self.vis.add_geometry(ds_pcd)
        # self.vis.poll_events()
        # self.vis.update_renderer()
        
        o3d.visualization.draw_geometries(ds_pcd)#点群を画像として表示
        #o3d.io.write_point_cloud("output.pcd", ds_pcd) #output.pcbという名前の点群ファイルを出力
        return ds_pcd

if __name__ == "__main__":
    PCM = PointCloudModule()
    PCM.rosInit(color=True)
    while True: 
        try:
            PCM.getHeadO3dColorPoints()
            time.sleep(5)
        except AttributeError as ae:
            print("AttributeError:" + str(ae))
            time.sleep(5) 
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
        except rospy.ROSInterruptException:
            print("ROSInterruptException")
