#!/usr/bin/env python3

"""
ROS server: return image information
IN: prompt, image==None | OUT: string result text
Requirements:
    pip3 install -q -U google-generativeai
    Google api key
References:
    https://note.com/mega_gorilla/n/nce4a2aa871c0
    https://qiita.com/yuwewe/items/ac0fb760602561f2531a
Author: washio
"""

import os
import yaml
from pathlib import Path
import textwrap
from IPython.display import Markdown
import google.generativeai as genai
from vertexai.preview import generative_models
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

import rospy, rosparam
import cv_bridge
from sensor_msgs.msg import Image
import prompt_module
from happymimi_recognition_msgs import UorVlm, UorVlmResponse

# IN: Sub(Yolo-world Results /uor/yolo_result) | OUT: Server (detecttion and localizeation, finding)
class GeminiServer():
    def __init__(self):
        rospy.init_node("uor_gemini_server")
        rospy.loginfo("Initializing Node: ** uor_gemini_server **")
        self.gmn_ss = rospy.Service("/uor/gemini_server", UorVlm, self.serviceCB)
        self.image_sc = rospy.ServiceProxy("/image_server", Image)
        #self.IM = ImageModule(); self.IM.rosInit()
        self.bridge = cv_bridge.CvBridge()
        self.genai_cfg = self.getConfig()
        self.model = genai.GenerativeModel(model_name=self.genai_cfg["model"])
        self.chat = self.model.start_chat()
        rospy.loginfo("GeminiServer: Im Ready to response...")
        
    def convertRosCv(self, ros_image): return self.bridge.imgmsg_to_cv2(ros_image)
    def getConfig():
        pkg_dir = Path(__file__).parent.resolve().parent
        with open(pkg_dir/"config/uor_model.yaml", 'r') as file:
            uor_model_config = yaml.safe_load(file)
        return uor_model_config
    def serviceCB(self, req):
        SrvRes = UorVlmResponse()
        #ros_img = IM.converRosJpg(req.image)
        if req.image is None:
            if req.camera_name == "head" or req.camera_name == None:
                image = self.image_sc("head") #cv_img = IM.getHeadCvImage()
            if req.camera_name == "arm":
                image = self.image_sc("arm") #cv_img = IM.getArmCvImage()
            if req.camera_name == "usbcam":
                image = self.image_sc("usbcam") # cv_img = IM.getUsbCvImage()
        else: image = req.image
        image = self.convertRosCv(image)
        if req.task == "clasification":
            self.clasification(req.prompt, image)
        if req.task == "detection":
            
        if req.task == "description":
        if req.task == "NTP":
            

        result = generateImageText(image=base_img, prompt=req.prompt)
        SrvRes.result = result
        return SrvRes
    def debug():# load image, exec it
        pass
    def clasification(self, image, list_name==None, object==None)
    def generateImageText(self, image, prompt): return self.model.generate_content([prompt, image])

class geminiSettings:
    safety_settings_NONE=[
        { "category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_NONE" },
        { "category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_NONE" },
        { "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_NONE" },
        { "category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_NONE"}
    ]
    safety_settings_ALL_HIGH=[
        { "category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_ONLY_HIGH" },
        { "category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_ONLY_HIGH" },
        { "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_ONLY_HIGH" },
        { "category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_ONLY_HIGH"}
    ]
    config = {
        "max_output_tokens": 300,
        "temperature": 1,
        "top_p": 1
    }

def generateText(prompt, gen_cfg, safety_settings):
    try:
       response = chat.send_message(content=prompt,
                                    generation_config=gen_cfg,
                                    safety_settings=safety_settings)
       print(response.text)
    except Exception as e:
        print(f"ERROR: {e}")
        if 'response' in locals():
            print(f"\n\nPrompt Feedback: {response.prompt_feedback}\n\nResponse Dict:\n{response.__dict__}")
    print("-" * 50)
    return response.text

def executeNoneSafety(prompt):
    print("----Response (None safety)----")
    chat = model.start_chat()
    res_text = generateText(prompt, geminiSettings.config, geminiSettings.safety_settings_NONE)

def executeHighSafety(prompt):
    print("----Response (High safety)----")
    chat = model.start_chat()
    res_text = generateText(prompt, geminiSettings.config, geminiSettings.safety_settings_ALL_HIGH)

def formatToJpMarkdown(text):
  text = text.replace('•', '  *')
  return Markdown(textwrap.indent(text, '> ', predicate=lambda _: True))

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
IM.rosInit(usb_cam=True)

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
    if req.camera_name == "arm":
        cv_img = IM.getArmCvImage()
    if req.camera_name == "usbcam":
        cv_img = IM.getUsbCvImage()
    else: cv_img = IM.autoConvert(cv_img, "cv")
    #base_img = IM.convertCvBase64(cv_img) ###
    image_path = IM.saveCvJpeg(cv_img)
    base_img=IM.readImageBase64(image_path)
    
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