#!/usr/bin/env python3

# ROS server: return image information
# IN: prompt, image==None | OUT: string result text

import os
import yaml
import json
from pathlib import Path
from PIL import Image
api_key_in = os.environ.get("OPEN_AI_KEY", "default")
from openai import OpenAI
# openai.api_key=
client = OpenAI(api_key=api_key_in)
import base64
import requests

import sys
import roslib
base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
sys.path.insert(0, base_path)
from image_module import ImageModule
from prompt_module import PromptModule
import rospy

from happymimi_recognition_msgs.srv import UorGpt, UorGptResponse

# テスト画像ファイルの準備
parent_dir = Path(__file__).parent.resolve()
img_dir = parent_dir.parent/"image"
img_file = img_dir/"pcp_2.png"
for filename in os.listdir(img_dir):
    if filename.endswith('.png'):  # 画像ファイルの拡張子を指定
        image_path = os.path.join(img_dir, filename)
        image = Image.open(image_path) # 画像を読み込み

def requestGpt(image, prompt=None): #{base64_image}"
    base_image = image
    if prompt is None: prompt = "What is this? output simply"
    headers = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {api_key_in}"
    }
    payload = {
    "model": "gpt-4-vision-preview",
    "messages": [
        {
        "role": "user",
        "content": [
            {
            "type": "text",
            "text": {prompt}
            },
            {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64, {base_image}"
            }
            }
        ]
        }
    ],
    "max_tokens": 100
    }
    ###
    payload["messages"][0]["content"][1]["image_url"]["url"] = str(payload["messages"][0]["content"][1]["image_url"]["url"])
    payload["messages"][0]["content"][0]["text"] = str(payload["messages"][0]["content"][0]["text"])
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
    print(response.json())
    result = response.json()["choices"][0]["message"]["content"]
    print("\nGPT result: "+result); return result

IM = ImageModule()

def encodeImage(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')
#image_path = "path_to_your_image.jpg"
# Getting the base64 string
base64_image = encodeImage(img_file)

def serviceCB(req):
    SrvRes = UorGptResponse()
    #ros_img = IM.converRosJpg(req.image)
    if req.camera_name == "head":
        cv_img = IM.getHeadCvImage()
    else: cv_img = IM.autoConvert(cv_img, "cv")
    base_img = IM.convertCvBase64(cv_img)
    result= requestGpt(image=base_img, prompt=req.prompt)
    SrvRes.result = result
    return SrvRes
    
def main():
    rospy.loginfo("Initialing Node: uor_gpt_server")
    rospy.init_node('uor_gpt_server')
    IM.rosInit()
    rospy.Service("/uor/gpt_server", UorGpt, serviceCB)
    rospy.loginfo("Initialized Node: uor_gpt_server")
    
    rospy.spin()

    
if __name__ == "__main__":
    main()