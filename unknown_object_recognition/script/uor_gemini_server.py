#!/usr/bin/env python3

"""
ROS server: return image information
IN: prompt, image==None | OUT: string result text
Requirements:
    pip3 install -q -U google-generativeai google-cloud-aiplatform
    Google api key
References:
    - https://note.com/mega_gorilla/n/nce4a2aa871c0
    - https://qiita.com/yuwewe/items/ac0fb760602561f2531a
    - https://github.com/GoogleCloudPlatform/generative-ai.git
Author: washio
"""

import os, sys
import yaml
from pathlib import Path
import textwrap
from IPython.display import Markdown
import google.generativeai as genai
#from vertexai.preview import generative_models
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

import rospy, rosparam
import cv_bridge
from sensor_msgs.msg import Image
from happymimi_recognition_msgs.srv import UorVlm, UorVlmResponse, CameraImage
parent_path = Path(__file__).parent.resolve()
sys.path.insert(0, parent_path)
import prompt_module
# テスト画像ファイルの準備
from PIL import Image as PILImage
img_dir = parent_path.parent/"image"
img_file = img_dir/"pcp_2.png"
for filename in os.listdir(img_dir):
    if filename.endswith('.png'):  # 画像ファイルの拡張子を指定
        image_path = os.path.join(img_dir, filename)
        image = PILImage.open(image_path) # 画像を読み込み

# IN: Sub(Yolo-world Results /uor/yolo_result) | OUT: Server (detecttion and localizeation, finding)
class GeminiServer():
    def __init__(self):
        rospy.init_node("uor_gemini_server")
        rospy.loginfo("Initializing Node: ** uor_gemini_server **")
        self.gmn_ss = rospy.Service("/uor/gemini_server", UorVlm, self.serviceCB)
        self.image_sc = rospy.ServiceProxy("/image_server", CameraImage)
        #self.IM = ImageModule(); self.IM.rosInit()
        self.bridge = cv_bridge.CvBridge()
        self.genai_cfg = self.getConfig()
        self.model = genai.GenerativeModel(model_name=self.genai_cfg["model"])
        self.chat = self.model.start_chat()
        rospy.loginfo("GeminiServer: Im Ready to response...")
        
    def convertRosCv(self, ros_image): return self.bridge.imgmsg_to_cv2(ros_image)
    def getConfig(self):
        pkg_dir = Path(__file__).parent.resolve().parent
        with open(pkg_dir/"config/uor_model.yaml", 'r') as file:
            uor_model_config = yaml.safe_load(file)
        return uor_model_config
    
    def serviceCB(self, req):
        SrvRes = UorVlmResponse()
        #ros_img = IM.converRosJpg(req.image)
        if req.image is None:
            if req.camera_name == "head" or req.camera_name == None:
                image = self.image_sc("head", req.depth_musk) #cv_img = IM.getHeadCvImage()
            if req.camera_name == "arm":
                image = self.image_sc("arm", req.depth_musk) #cv_img = IM.getArmCvImage()
            if req.camera_name == "usbcam":
                image = self.image_sc("usbcam", req.depth_musk) # cv_img = IM.getUsbCvImage()
        else: image = req.image
        image = self.convertRosCv(image)
        if req.task == "clasification":
            self.clasification(req.prompt, image)
        if req.task == "detection": pass
        if req.task == "description": pass
        if req.task == "NTP": pass
        result = self.generateImageText(image=base_img, prompt=req.prompt)
        SrvRes.result = result
        return SrvRes
    def debug(self):# load image, exec it
        pass
    def clasification(self, image, list_name=None, object=None): pass
    def generateImageText(self, image, prompt): return self.model.generate_content([prompt, image])
    
    def executeNoneSafety(self, prompt):
        print("----Response (None safety)----")
        chat = self.model.start_chat()
        res_text = self.generateText(prompt, geminiSettings.config, geminiSettings.safety_settings_NONE)
        return res_text
    
    def executeHighSafety(self, prompt):
        print("----Response (High safety)----")
        chat = self.model.start_chat()
        res_text = self.generateText(prompt, geminiSettings.config, geminiSettings.safety_settings_ALL_HIGH)
        return res_text

    def formatToJpMarkdown(self, text):
        text = text.replace('•', '  *')
        return Markdown(textwrap.indent(text, '> ', predicate=lambda _: True))
    
    def generateText(self, prompt, gen_cfg, safety_settings):
        try:
            response = self.chat.send_message(content=prompt,
                                        generation_config=gen_cfg,
                                        safety_settings=safety_settings)
            print(response.text)
        except Exception as e:
            print(f"ERROR: {e}")
            if 'response' in locals():
                print(f"\n\nPrompt Feedback: {response.prompt_feedback}\n\nResponse Dict:\n{response.__dict__}")
        print("-" * 50)
        return response.text


class geminiSettings():
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

def main():
    GS = GeminiServer()
    rospy.spin()

if __name__ == "__main__":
    main()