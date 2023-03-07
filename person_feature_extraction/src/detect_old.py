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

class DetectAge(object):
    def __init__(self):
        self.srv = rospy.Service('/person_feature/old', SetStr, self.age_detection)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size=1)
        
        self.API_KEY = "YMdjJTAjw58SIJeuiPvj-tkp35GQUHya"
        self.API_SECRET = "dU1mXNRiGuCk_SKqYPrq4cVncps6HD9h"
        self.file_path = "screenshot.jpg"
        self.savefile = "result"
        
    def realsenseCB(self, res):
        self.image_res = res
    
    def parameter_read(self, _):
        
        json_open = open(self.savefile+".json", 'r')
        json_load = json.load(json_open)
            
        print(json_load["faces"][0]["attributes"]["age"]["value"])
        
        result_old = str(json_load["faces"][0]["attributes"]["age"]["value"])
        

        return result_old
        
    def age_detection(self, _):
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
        
        return response.result#Mask(=1) or Glass(=2)
        self.select = 1



    def getOld(self):
        self.getold_srv = rospy.ServiceProxy('/person_feature/old', SetStr, self.age_detection)
        self.old_year = 0

        self.old_year = int(self.getold_srv().result)

        if self.old_year < 20: return " looks under 20"
        elif self.old_year >= 20 and self.old_year < 30: return " looks in the twenties"
        elif self.old_year >= 30 and self.old_year < 40: return " looks in the 30s"
        elif self.old_year >= 40 and self.old_year < 50: return " looks in the 40s"
        elif self.old_year >= 50 and self.old_year < 60: return " looks in the 50s"
        elif self.old_year >= 60 and self.old_year < 70: return " looks in the 60s"

        else:
            return "so old!!" 

    def getheight(self):
        self.getheight_srv = rospy.ServiceProxy('/person_feature/height')
        self.height = 0

        self.height = int(self.getheight_srv().result)

        if self.height < 160: return " looks less than 160cm"
        elif self.height >= 160 and self.height < 170: return " looks like 160cm to 170cm"
        elif self.height >= 170 and self.height < 180: return " looks like 170cm to 180cm"
        elif self.height >= 180 and self.height < 190: return " looks like 180cm to 190cm"
        else:
            return "so big!"

        

if __name__ == '__main__':
    rospy.init_node('detect_glass')
    detect_age = DetectAge()
    # get command line arguments 
    rospy.spin()