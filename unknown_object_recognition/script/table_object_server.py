#!/usr/bin/env python3

# 点群で目の前のテーブル上にある物体の3次元位置、距離、形状、傾きなどをROSサービス通信で返すサービスサーバー
# IN: 
#   string scope
#   

import os
import yaml
import numpy as np
import ros_numpy
import open3d as o3d

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from happymimi_recognition_msgs.srv import UOR, UORResponse
import sys
import roslib
base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
sys.path.insert(0, base_path)
from pc_module import PointCloudModule


# IN: Points | OUT: UOR(centroid 3d coords, ,size, shape)
class TableObjectServer():
    def __init__(self, debug=False):
        rospy.init_node("table_object_server")
        rospy.loginfo("Initializing Node: ** table_object_server **")
        self.PCM = PointCloudModule()
        self.PCM.rosInit(color=True)
        self.to_ss = rospy.Service("/uor/table_object_server", UOR, self.serviceCB)
        self.show_o3d_ss = rospy.Service("/uor/show_o3d_server", Empty, self.showPcd)
        # DEBUG
        if debug: 
            rospy.loginfo("table_object_server: DEBUG Mode")
            self.pc2_pub = rospy.Publisher('/uor/processed_pointcloud', PointCloud2, queue_size=10)
            self.rate = rospy.Rate(2)  # パブリッシュの周波数
            #if self.PCM.o3d_pcd == None:
                #self.publishO3dPc(self.PCM.extractPointsOnTable()) #o3d_color_pcd))
            # self.PCM.
        
        rospy.loginfo("TableObjectServer: Im Ready to response...")
        
    def serviceCB(self, req):
        UORMSG = UORResponse()
        p = Point(); print("\n p:" + str(Point()))
        centroid_num, centroid_coords = self.PCM.extractTableObjects(self.PCM.o3d_color_pcd, req.distance)
        
    
    # DEBUG ------------------------------
    # Open3Dで処理した結果の色点群データをROSでパブリッシュする
    def publishO3dPc(self): # , o3d_pcd):
        while not rospy.is_shutdown():
            o3d_pcd = self.PCM.extractPointsOnTable(self.PCM.o3d_color_pcd)
            pc2_msg = self.PCM.convertO3dPc2(o3d_pcd)
            # Pub tf setting
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_link" # Set the appropriate frame ID
            # Convert the points to a PointCloud2 message
            np_pc2_msg = ros_numpy.point_cloud2.array_to_pointcloud2(pc2_msg, header)
            self.pc2_pub.publish(np_pc2_msg)
            # Sleep to control the publishing rate
            rospy.sleep(1.0)  # Adjust the publishing rate as needed
            
    def showO3dPoints(self, pcd):
        plane_pcd = o3d.io.read_point_cloud(pcd)
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(plane_pcd)
        vis.run()
    def showPcd(self):
        #self.showO3dPoints(self.PCM.o3d_color_pcd)
        while True:
            print("self.PCM.o3d_color_pcd" + self.PCM.o3d_color_pcd)
            rospy.sleep(0.5)
        
    def debugPlane(self, option):
        if option == "objects_cloud":
            objects_points = self.PCM.extractPointsOnTable(self.PCM.o3d_color_pcd)
        

def main():
    TOS = TableObjectServer(debug=True)
    
    #TOS.showPcd()
    rospy.spin()

if __name__ == "__main__":
    while True: 
        try:
            #main()
            PCM=PointCloudModule()
            PCM.rosInit(color=True)
            o3d_color_pc = PCM.getHeadO3dColorPoints()
            rospy.sleep(3.0)
        except AttributeError as ae:
            print("AttributeError:" + str(ae))
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
        except rospy.ROSInterruptException:
            print("ROSInterruptException")