#!/usr/bin/env python3

# ROS server: return image information

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
from uor_modele import ImageHub

# 画像ファイルの準備
parent_dir = Path(__file__).parent.resolve()
img_dir = parent_dir.parent/"image"
img_file = img_dir/"pcp_2.png"
#img_dir = "./../image/"
for filename in os.listdir(img_dir):
    if filename.endswith('.png'):  # 画像ファイルの拡張子を指定
        image_path = os.path.join(img_dir, filename)
        image = Image.open(image_path) # 画像を読み込み
# Function to encode the image
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')
#image_path = "path_to_your_image.jpg"
# Getting the base64 string
base64_image = encode_image(img_file)


def requestGpt(self, prompt==None, image):
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
            "text": "What’s in this image?"
            },
            {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}"
            }
            }
        ]
        }
    ],
    "max_tokens": 300
    }
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
    print(response.json())

    print(response.json()["choices"][0]["message"]["content"])

IM = ImageHub()

def serviceCB(req):
    i = IM.converRosJpg(req.image)
    res= requestGpt(image=i)
    return res
    
def main():
    rospy.init_node('uor_gpt_server')
    rospy.Service("/uor_gpt_server", UorGpt, serviceCB)

    
if __name__ == "__main__":
    main()