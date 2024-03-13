#!/usr/bin/env python3

# IN: format(jpeg, png, cv:Mat, sensor_msgs/Image,,,), camera(head, arm,,,)
# OUT: image_data

import rospy
from happymimi_recognition_msgs.srv import ImageServer
from uor_module import Image
IMAGE = Image

def main():
    rospy.init_node('image_saver')
    rospy.Service("/recognition/image_saver", )
    rospy.spin()


if __name__ == '__main__':
    main()