#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
from subprocess import PIPE
import subprocess
import math
import numpy as np
import pathlib 
from pathlib import Path
import base64
# Process API --------------------------------
import cv2
import torch
import open3d as o3d
import matplotlib.pyplot as plt 
# Remote API --------------------------------
import requests
from transformers import pipeline
import clip
from PIL import Image as PILImage
from transformers import CLIPProcessor, CLIPModel ###
#----------------------------------------------------------------
from sklearn.cluster import DBSCAN ###
# Local API ----------------------------------------------------
from transformers import BlipProcessor, BlipForConditionalGeneration #Instrtion: https://github.com/salesforce/BLIP
from ultralytics import YOLOWorld
# ROS --------------------------------
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
# ROS Custum Messages --------------------------------
from happymimi_msgs.srv import SetStr, SetStrResponse

password = (os.environ["SUDO_KEY"] + "\n").encode()
pkg_dir = Path(__file__).parent.resolve().parent
image_dir = pkg_dir/"image/"
# with open(pkg_dir/"config/uor_model.yaml", 'r') as file:
#     uor_model_config = yaml.safe_load(file)
# def getDevice():
#     device = "cuda" if torch.cuda.is_available() else "cpu"
#     if device == "cuda":
#         if uor_model_config["device"] == "gpu": pass
#         else: device = "cpu"
#     return device
# device = getDevice()







# import argparse
# import base64
# import settings
# from settings.setting import API_KEY

# class Gpt4vHub():
#     def __init__():
#         print("Initializing GPT4vHub Object...")
    
#     def parseArgs():
#         parser = argparse.ArgumentParser()
#         parser.add_argument('-i', '--image', type=str,)
#         parser.add_argument('-p', '--prompt', type=str)
#         return parser.parse_args()
    
#     def encodeImage(self, image_file):
#         if image_file is str():
#             if "/" in text:
#                 with open(image_path, "rb") as image_file:
#                     return base64.b64encode(image_file.read()).decode('utf-8')
#             else: return None
#         return base64.b64encode(image_file.read()).decode('utf-8')
    