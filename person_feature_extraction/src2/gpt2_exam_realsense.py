#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#GPTのCLIPから特徴量推定を行う
#ROSとの連携を考慮し、画像取得はRealsenseで行う
from PIL import Image
import requests
from transformers import CLIPProcessor, CLIPModel
#モデルの読み込み
model = CLIPModel.from_pretrained("openai/clip-vit-large-patch14")
processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")

#ROSやCV2の用意
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from happymimi_msgs.srv import SetStr, SetStrResponse
from happymimi_recognition_msgs.srv import Clip, ClipResponse

class Person_extract(object):
    def __init__(self) -> None:
        #rospy.Service('/person_feature/gpt', SetStr, self.main)
        rospy.Service('/person_feature/gpt', Clip, self.main)
        rospy.Subscriber('/camera/color/image_raw', Image, self.realsenseCB)
        self.bridge = CvBridge()
        # テキスト考案中...
        self.label_cloth_color = ["person dressed in white", "person dressed in red"]
        self.label_pants_color = ["person wearing white pants", "person wearing black pants"]
        #取り敢えず試験用に特徴量２つ
        self.label_gender = ["a photo of a man", "a photo of a woman"]
        self.label_glass = ["a photo of a man wearing glass", "a photo of a man without glasses"]
    
    def realsenseCB(self, res):
        self.image_res = res
    
    def extract_gender(self):
        #試験的に性別を判断する
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_gender = processor(text=self.label_gender, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_gender = model(**inputs_gender)
        logits_per_image = outputs_gender.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_gender[predicted_class_idx])
        print("score:", probs)
        
        return self.label_gender[predicted_class_idx]
    
    def extract_glass(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_glass = processor(text=self.label_glass, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_gender = model(**inputs_glass)
        logits_per_image = outputs_gender.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_glass[predicted_class_idx])
        print("score:", probs)
        
        return self.label_glass[predicted_class_idx]
    
    def main(self, request):
        #response = SetStrResponse()
        data = request.data
        response = ClipResponse()
        if data == "gender":response.result = str(self.extract_gender())
        elif data == "glass":response.result = str(self.extract_glass())
        elif data == "":
            rospy.loginfo("no select data")
            response.result = "False"
            return response
        
        return response

if __name__ == '__main__':
    rospy.init_node('person_extract')
    #image_saver = ImageSaver()
    person_extract = Person_extract()
    rospy.spin()


