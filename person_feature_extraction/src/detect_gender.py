#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import requests
import subprocess as sp
import time
import base64
import json
import os, sys
import math
import pyautogui

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
#from happymimi_msgs.srv import SetStr, SetStrResponse
from happymimi_msgs.srv import SetStr, SetStrResponse

class DetectGender(object):
    def __init__(self):
        self.srv = rospy.Service('/person_feature/gender', SetStr, self.gender_detection)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)
        
        self.API_KEY = "YMdjJTAjw58SIJeuiPvj-tkp35GQUHya"
        self.API_SECRET = "dU1mXNRiGuCk_SKqYPrq4cVncps6HD9h"
        self.file_path = "screenshot.jpg"
        self.savefile = "result"
        #Mask(=1) or Glass(=2)
        self.select = 1

    def realsenseCB(self, res):
        self.image_res = res
    
    def parameter_read(self, _):
        
        json_open = open(self.savefile+".json", 'r')
        json_load = json.load(json_open)
            
        print(json_load["faces"][0]["attributes"]["gender"]["value"])
        
        result_gender = json_load["faces"][0]["attributes"]["gender"]["value"]
        

        return result_gender
        
    def gender_detection(self, _):
        response = SetStrResponse()

        #画像の取得
        s = pyautogui.screenshot()
        s.save('screenshot.jpg')
        time.sleep(2.0)

        #Get API key
        #API_KEY = ""
        #API_SECRET = ""  
        
        #Set image file and save file
        #file_path = "screenshot.jpg"
        #savefile = "result"
        
        #Open imagefile with binary (base64)
        with open(self.file_path, 'rb') as f:
            img_in = f.read()
        
        img_file = base64.encodebytes(img_in)
            
        #URL for Web API    
        url = 'https://api-us.faceplusplus.com/facepp/v3/detect'
        
        #Set configuration
        config = {'api_key':self.API_KEY,
                'api_secret':self.API_SECRET,
                'image_base64':img_file,
                'return_landmark':1,
                'return_attributes':'gender,age,smiling,headpose,facequality,blur,eyestatus,emotion,ethnicity,beauty,mouthstatus,eyegaze,skinstatus'}
        
        # POST to Web API
        res = requests.post(url, data=config)
        
        # Load json data
        data = json.loads(res.text)
        
        # Save json data to 'savefile'
        with open(self.savefile+'.json', 'w') as f:
            json.dump(data, f, indent=4)
        
        response.result = self.parameter_read(self.savefile)
        
        return response.result

if __name__ == '__main__':
    rospy.init_node('detect_glass')
    detect_gender = DetectGender()
    # get command line arguments 
    rospy.spin()