#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#GPTのCLIPから特徴量推定を行う
#ROSとの連携を考慮し、画像取得はRealsenseで行う
from PIL import Image as PImage
import requests
from transformers import CLIPProcessor, CLIPModel
import clip
import numpy as np
import torch

device = "cpu"
model, preprocess = clip.load("RN50x64", device=device)
#モデルの読み込み
#model = CLIPModel.from_pretrained("openai/clip-vit-large-patch14")
#processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")

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
        
        self.label_sit = ["A person are sitting in front of the chair on the right.",
                          "A person are sitting in front of the chair on the left."]
        
        self.label_chair = ["The chair on the right is empty.",
                            "The chair on the left is empty."]
        
        rospy.loginfo("Ready server CLIP")
    
    def realsenseCB(self, res):
        self.image_res = res
    
    def extract_gender(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_sit).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_sit[max_index]
    
    def extract_glass(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_glass).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_glass[max_index]
    
    def extract_cloth_color(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_cloth_color).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_cloth_color[max_index]
    
    def extract_pants_color(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_pants_color).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_pants_color[max_index]
    
    def extract_hair_color(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_hair_color).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_hair_color[max_index]
    
    def extract_age(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_age).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_age[max_index]
    
    def extract_sit(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_sit).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_sit[max_index]
        
    def extract_chair(self):
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        np_image = np.array(image)
        # NumPy配列をPILのイメージに変換
        pil_image = PImage.fromarray(np_image)
        image = preprocess(pil_image).unsqueeze(0).to(device)
        text = clip.tokenize(self.label_chair).to(device)
        
        with torch.no_grad():
            image_features = model.encode_image(image)
            text_features = model.encode_text(text)

            logits_per_image, logits_per_text = model(image, text)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
        print("Label probs:", probs)
        probs_array = np.array(probs)
        # 最大値のインデックスを取得
        max_index = np.argmax(probs_array)
        return self.label_chair[max_index]
    
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
        elif data == "sit":response.result = str(self.extract_sit())
        elif data == "chair":response.result = str(self.extract_chair())
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


