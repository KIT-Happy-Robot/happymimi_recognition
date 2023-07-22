#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#GPTのCLIPから特徴量推定を行う
#ROSとの連携を考慮し、画像取得はRealsenseで行う
from PIL import Image as PILImage
import requests
from transformers import pipeline

#モデルの読み込み
captioner = pipeline("image-to-text",model="Salesforce/blip-image-captioning-base")

#ROSやCV2の用意
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from happymimi_msgs.srv import SetStr, SetStrResponse
from happymimi_recognition_msgs.srv import ITT, ITTResponse

import os
from subprocess import PIPE
import subprocess

password = (os.environ["SUDO_KEY"] + "\n").encode()

#プロキシ対策
def check_wifi():
    proc = subprocess.run(["sudo","-S","wpa_cli", "status"],stdout = subprocess.PIPE, stderr = subprocess.PIPE, input=password)
    data = proc.stdout.decode("utf8").split()
    if data[5] == "ssid=KIT-WLAP2":
        server = "http://wwwproxy.kanazawa-it.ac.jp:8080"
        servers = "https://wwwproxy.kanazawa-it.ac.jp:8080"
        os.environ["http_proxy"] = server
        os.environ["https_proxy"] = servers
        
    else:
        server = ""
        os.environ["http_proxy"] = server
        os.environ["https_proxy"] = server

class Image_To_Text(object):
    def __init__(self) -> None:
        #rospy.Service('/person_feature/gpt', SetStr, self.main)
        rospy.Service('/image_to_text/image', ITT, self.main)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        self.bridge = CvBridge()
        
    def realsenseCB(self, res):
        self.image_res = res
    
    def extract(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        pil_image = PILImage.fromarray(image)
        result = str(captioner(pil_image)[0]["generated_text"]) 
        
        return result

    def main(self, request):
        response = ITTResponse()
        response.result = self.extract()
        return response
        
      

if __name__ == '__main__':
    rospy.init_node('image_to_text')
    rospy.loginfo("ready to ITT")
    #check_wifi()
    ITT = Image_To_Text()
    rospy.spin()