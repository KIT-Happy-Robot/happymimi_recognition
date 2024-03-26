#!/usr/bin/env python3

# ROS server: return image information
# IN: prompt, image==None | OUT: string result text

import os
import yaml
import json
import cv2
from pathlib import Path
from PIL import Image
api_key_in = os.environ.get("OPEN_AI_KEY", "default")
from openai import OpenAI
# openai.api_key=
client = OpenAI(api_key=api_key_in)
import base64
import requests

import sys
parent_path = Path(__file__).parent.resolve()
sys.path.insert(0, parent_path)
from image_module import ImageModule
from prompt_module import PromptModule
import rospy
from happymimi_recognition_msgs.srv import UorVlm, UorVlmResponse

# テスト画像ファイルの準備
img_dir = parent_path.parent/"image"
img_file = img_dir/"pcp_2.png"
for filename in os.listdir(img_dir):
    if filename.endswith('.png'):  # 画像ファイルの拡張子を指定
        image_path = os.path.join(img_dir, filename)
        image = Image.open(image_path) # 画像を読み込み

class UnknownObjectGptServer():
    def __init__(self):
        rospy.loginfo("Initialing Node: uor_gpt_server")
        rospy.init_node('uor_gpt_server')
        rospy.Service("/uor/gpt_server", UorVlm, self.serviceCB)
        self.image_sc = rospy.ServiceProxy("/recognition/image_saver", Image)
        rospy.loginfo("Initialized Node: uor_gpt_server")
        self.IM = ImageModule()
        #image_path = "path_to_your_image.jpg"
        # Getting the base64 string
        self.base64_image = self.encodeImage(img_file) # TEST

    def requestGpt(self, image, prompt=None): #{base64_image}"
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

    def encodeImage(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def saveBase64(self, cv_image):
        try:
            base64_image = base64.b64encode(cv2.imencode('.jpg', cv_image)[1]).decode()
            with open('gpt_base64.jpg', 'wb') as f:
                f.write(base64.b64decode(base64_image))
            cv2.imwrite('gpt_image.jpg', cv_image)
            rospy.loginfo("Images saved successfully!")
        except Exception as e:
            rospy.logerr("Error processing the image: %s", str(e))

    def serviceCB(self, req):
        SrvRes = UorVlmResponse()
        #ros_img = IM.converRosJpg(req.image)
        if req.camera_name == "head":
            cv_img = self.image_sc() #IM.getHeadCvImage()
        if req.camera_name == "arm":
            cv_img = self.image_sc() #IM.getArmCvImage()
        if req.camera_name == "usbcam":
            cv_img = IM.getUsbCvImage()
        else: cv_img = IM.autoConvert(cv_img, "cv")
        #base_img = IM.convertCvBase64(cv_img) ###
        image_path = self.IM.saveCvJpeg(cv_img)
        base_img=self.IM.readImageBase64(image_path)
        
        result= self.requestGpt(image=base_img, prompt=req.prompt)
        SrvRes.result = result
        return SrvRes
    
def main():
    UOGS = UnknownObjectGptServer()
    rospy.spin()

    
if __name__ == "__main__":
    main()