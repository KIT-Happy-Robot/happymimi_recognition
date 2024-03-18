#!/usr/bin/env python3
#
#
import rospy
import roslib
base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
sys.path.insert(0, base_path)
from image_module import ImageModule
from prompt_module import PromptModule

import os
import sys
import yaml
from pathlib import Path
from subprocess import PIPE
import subprocess
from PIL import Image as PILImage
import torch
# Remote API --------------------------------
import clip
from transformers import pipeline
from transformers import CLIPProcessor, CLIPModel ###

class ClipModule():
    def __init__(self):
        print("Initializing CLIP")
        parent_dir = Path(__file__).parent.resolve()
        pkg_dir = parent_dir.parent
        with open(pkg_dir/"config/uor_model.yaml", 'r') as file:
            self.uor_model_config = yaml.safe_load(file)
        self.checkWifi()
        # DIVECE setting
        self.device = self.getDevice()
        self.setConfig()
        self.loadModel()
    def checkWifi(self):
        password = (os.environ["SUDO_KEY"] + "\n").encode()
        proc = subprocess.run(["sudo","-S","wpa_cli", "status"],stdout = subprocess.PIPE, stderr = subprocess.PIPE, input=password)
        data = proc.stdout.decode("utf8").split()
        print(f"checkWifi: ssid={data[5]}")
        if data[5] == "ssid=KIT-WLAP2":
            server = "http://wwwproxy.kanazawa-it.ac.jp:8080"
            #servers = "https://wwwproxy.kanazawa-it.ac.jp:8080"
            os.environ["http_proxy"] = server
            os.environ["HTTP_PROXY"] = server
            os.environ["https_proxy"] = server
            os.environ["HTTPS_PROXY"] = server
        else:
            server = ""
            os.environ["http_proxy"] = server
            os.environ["https_proxy"] = server
            os.environ["https_proxy"] = server
            os.environ["HTTPS_PROXY"] = server
    def setConfig(self):
        pkg_dir = Path(__file__).parent.resolve().parent
        with open(pkg_dir/"config/uor_model.yaml", 'r') as file:
            self.uor_model_config = yaml.safe_load(file)
    def getDevice(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        if self.device == "cuda":
            if self.uor_model_config["device"] == "gpu": pass
            else: self.device = "cpu"
        return self.device
    #モデルの読み込み ###
    def loadModel(self, task="clasification"):
        if task=="clasification":
            print("\nClipHub: Complete loading model...")
            #self.model, self.preprocess = clip.load(self.uor_model_config["clip"]["model"], device=self.device)
            self.model = CLIPModel.from_pretrained("openai/clip-vit-large-patch14")
            self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")
            print("ClipHub: Complete loading model")
            # memo
            #image = preprocess(Image.open("test.png")).unsqueeze(0).to(self.device)
            #text = clip.tokenize(["a human", "a dog", "a cat"]).to(self.device)
        #if task=="description": ###
        #captioner = pipeline("image-to-text",model="Salesforce/blip-image-captioning-base")
        #processor = CLIPProcessor.from_pretrained("openai/clip-vit-large-patch14")

    def objectDetection(self): pass # Out:Bool
    def objectNameClasifiction(self, labels, image): # IN:text list, cv image | OUT:max prop
        # clip
        # image_features = self.model.encode_image(image)
        # text_features = self.model.encode_text(text_list)
        #logits_per_image, logits_per_text = self.model(image, labels)# 推論
        # CLIP
        inputs = self.processor(text=labels,
                                images=image, 
                                return_tensors='pt', 
                                padding=True)
        outputs = self.model(**inputs)
        logits_per_image = outputs.logits_per_image
        logits_per_text = outputs.logits_per_text
        probs = logits_per_image.softmax(dim=1); print("\nClipHub: probs: "+probs)
        top_class= probs.argmax(-1).item(); print("\nClipHub: top class: "+ top_class)
        return

    def objectColorClasification(self): pass
    def objectCategoryClasification(self): pass
    def binaryClasification(self): pass
    def extract_gender(self):
        #試験的に性別を判断する
        image = self.bridge.imgmsg_to_cv2(self.image_res)
        inputs_gender = self.processor(text=self.label_gender, images=image,
                        return_tensors="pt", padding=True)

        outputs_gender = self.model(**inputs_gender)
        logits_per_image = outputs_gender.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_gender[predicted_class_idx])
        print("score:", probs)

        return self.label_gender[predicted_class_idx]
