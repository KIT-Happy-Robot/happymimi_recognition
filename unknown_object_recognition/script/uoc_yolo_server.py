#!/usr/bin/env python3

# Yolo-worldで特定のラベルのリストの情報に従って、
# 未知の物体の位置、物体名などの情報を返すROSサービスサーバーノード
# uor_moduleのYoloクラスやその関数を使って、未知物体のバウンディングボックスを取得やラベル名の取得をする。
# IN: 

import rospy
from happymimi_recognition_msgs import UorYolo, UorYoloResponse
from uor_module import ImageHub, YoloHub


class UnkownObjectYoloServer(ImageHub):
    def __init__(self):
        self.uor_yolo_ss = rospy.Service("/uor/yolo_server", )
        rospy.loginfo("Initializing Node: ** Unknown object Clip Server **")
        
    def serviceCB()

    

# 