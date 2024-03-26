#!/usr/bin/env python3

"""
ヘッド・アームのRSカメラ画像をサービスで取得するためのサービスサーバー
IN: camera(head, arm,,,), depth_musk(t/f, distance_value)
OUT: image_data
"""

import sys
import rospy
from sensor_msgs.msg import Image
from pathlib import Path
import roslib
from happymimi_recognition_msgs.srv import CameraImage, CameraImageResponse
# parent_path = Path(__file__).parent.resolve()
# sys.path.insert(0, parent_path)
base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
sys.path.insert(0, base_path)
from image_module import ImageModule

class ImageServer():
    def __init__(self):
        rospy.loginfo("\nInitializing Image Server...")
        rospy.init_node('image_saver')
        rospy.Service("/recognition/image_saver", CameraImage, self.serviceCB)
        self.IM = ImageModule()
        self.IM.rosInit(arm=True, arm_depth=True, usb_cam=True,head_depth=True, depth_musk=True)
        rospy.loginfo("\nImage Server: I'm ready ...")

    def serviceCB(self, req):
        if req.camera_name == "head" or req.camera_name == None:
            image = self.IM.head_color_image
            if req.depth_musk: image = self.IM.head_depth_musk_color_image
        if req.camera_name == "head_musk":
            image = self.IM.head_depth_musk_color_image
        if req.camera_name == "usbcam":
            image = self.IM.usb_cam_image
        return image

if __name__ == '__main__':
    try:
        IS = ImageServer()
        rospy.spin()
    except:
        pass
