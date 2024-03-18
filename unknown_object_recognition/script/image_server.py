#!/usr/bin/env python3

# いろんなフォーマットでおでこ、アームのRSカメラ画像をサービスで取得するためのサービスサーバー
# IN: format(jpeg, png, cv:Mat, sensor_msgs/Image,,,), camera(head, arm,,,), depth_musk(t/f, distance_value)
# OUT: image_data

import rospy
from happymimi_recognition_msgs.srv import ImageServer
from image_module import ImageModule
IM = ImageModule

def serviceCB(req):
    
def main():
    rospy.init_node('image_saver')
    rospy.Service("/recognition/image_saver", ImageServer, serviceCB)
    
    rospy.spin()


if __name__ == '__main__':
    main()