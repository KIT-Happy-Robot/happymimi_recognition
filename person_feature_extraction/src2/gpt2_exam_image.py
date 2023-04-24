#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#GPTのCLIPから特徴量推定を行う
#ROSとの連携を考慮し、画像取得はRealsenseで行う
#from PIL import Image, ImageFilter
import PIL.Image
import requests
from transformers import CLIPProcessor, CLIPModel
#モデルの読み込み
model = CLIPModel.from_pretrained("openai/clip-vit-large-patch14")
processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")

#保存する画像ファイルの名前
file_name = "color_image.jpg"
#ROSやCV2の用意
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSaver:
    def __init__(self, image):
        # RealSenseカメラの画像を受信するSubscriberの設定
        self.image = image
    
    def callback(self):
        # 受信した画像データをOpenCVの画像形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        # 画像の保存（PNG形式）
        # #cv2.imwrite('color_image.png', cv_image)
        # # 画像の保存（JPG形式）
        cv2.imwrite(file_name, cv_image)



class Person_extract:
    def __init__(self, image_file) -> None:
        self.image_file = PIL.Image.open(image_file)
        # テキスト考案中...
        self.label_cloth_color = []
        self.label_pants_color = []
        #取り敢えず試験用に特徴量２つ
        self.label_gender = ["a photo of a man", "a photo of a woman"]
        self.label_glass = ["a photo of a man wearing glass", "a photo of a man without glasses"]
    
    def extract(self):
        #試験的に性別を判断する
        inputs_gender = processor(text=self.label_gender, images=self.image_file,
                        return_tensors="pt", padding=True)
        outputs_gender = model(**inputs_gender)
        logits_per_image = outputs_gender.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("-----------------------------------------------")
        print("class:",self.label_gender[predicted_class_idx])
        print("score:", probs)

if __name__ == '__main__':
    #rospy.init_node('image_saver')
    #image_saver = ImageSaver()
    #image_saver.callback(file_name)
    
    #rospy.Subscriber('/camera/depth/image_raw', Image, callback)
    person_extract = Person_extract(file_name)
    person_extract.extract()
    #rospy.spin()


