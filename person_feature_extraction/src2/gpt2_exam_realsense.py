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
        self.label_cloth_color = ["person wearing white clothes", "person wearing red clothes",
                                  "person wearing blue clothes","person wearing black clothes",
                                  "person wearing glay clothes","person wearing brown clothes",
                                  "person wearing orange clothes", "person wearing yellow clothes",
                                  "person wearing green clothes","person wearing purple clothes",
                                  "person wearing pink clothes"]
        
        self.label_pants_color = ["person wearing white pants", "person wearing black pants"]
        
        self.label_hair_color = ["dark-haired person","white-haired person",
                                 "person with brown hair","red-haired person"]
        
        self.label_gender = ["a photo of a man", "a photo of a woman"]
        self.label_glass = ["a photo of a person wearing glasses", "a photo of a person not wearing glasses"]

        self.label_age = ["This person appears to be in his/her 10s.",
                          "This person appears to be in his/her 20s.",
                          "This person appears to be in his/her 30s.",
                          "This person appears to be in his/her 40s.",
                          "This person appears to be in his/her 50s.",
                          "This person appears to be in his/her 60s.",
                          "This person appears to be in his/her 70s."]
    
    
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
    
    def extract_cloth_color(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_cloth = processor(text=self.label_cloth_color, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_cloth = model(**inputs_cloth)
        logits_per_image = outputs_cloth.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_cloth_color[predicted_class_idx])
        print("score:", probs)
    
        return self.label_cloth_color[predicted_class_idx]
    
    def extract_pants_color(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_pants = processor(text=self.label_pants_color, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_pants = model(**inputs_pants)
        logits_per_image = outputs_pants.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_pants_color[predicted_class_idx])
        print("score:", probs)
    
        return self.label_pants_color[predicted_class_idx]
    
    def extract_hair_color(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_hair = processor(text=self.label_hair_color, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_hair = model(**inputs_hair)
        logits_per_image = outputs_hair.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_hair_color[predicted_class_idx])
        print("score:", probs)
    
        return self.label_hair_color[predicted_class_idx]
    
    def extract_age(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_age = processor(text=self.label_hair_color, images=image,
                        return_tensors="pt", padding=True)
        
        outputs_age = model(**inputs_age)
        logits_per_image = outputs_age.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_age[predicted_class_idx])
        print("score:", probs)
    
        return self.label_age[predicted_class_idx]
    
    
    def main(self, request):
        #response = SetStrResponse()
        data = request.data
        response = ClipResponse()
        if data == "gender":response.result = str(self.extract_gender())
        elif data == "glass":response.result = str(self.extract_glass())
        elif data == "cloth" :response.result = str(self.extract_cloth_color())
        elif data == "pants":response.result = str(self.extract_pants_color())
        elif data == "hair":response.result = str(self.extract_hair_color())
        elif data == "age":response.result = str(self.extract_age())
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


