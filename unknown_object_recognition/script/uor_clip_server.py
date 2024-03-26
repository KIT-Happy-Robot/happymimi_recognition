#!/usr/bin/env python3

# CLIPで
# IN: task_name, target(obj, person, env), target_area(center, all,,,), class_list[], image
# OUT:clacified_name

import rospy
from happymimi_recognition_msgs import UorClip, UorClipResponse
import roslib
import sys
base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
sys.path.insert(0, base_path)
from image_module import ImageModule
from clip_module import ClipModule

class UnknownObjectClipServer():
    def __init__(self):
        rospy.loginfo("Initializing Node: ** Unknown object Clip Server **")
        rospy.Service("/uro/clip_server", UorClip, self.serviceCB)
        self.IM = ImageModule(); self.IM.rosInit()
        self.CM = ClipModule()
        self.object_class_list = rospy.get_param("/object_class_list", [])
        rospy.loginfo("UnknownObjectClipServer: Im Ready to response...")
        
    def serviceCV(self, req):
        image = self.IM.autoConvert(req.image, "cv")
        task_name = req.task_name
        target_name = req.target_name
        target_area = "center" if req.target_area is None else req.target_area
        class_list = self.object_class_list["default"] if req.class_list is None else req.class_list

        if task_name is None or task_name == "classification":
            result = self.executeMultiClassClassification(target_name, target_area, )
    
    def executeMultiClassClassification(self, ):
        prompt = 
        process_data = 
        image = 
        self.CLIPHUB
        
        # official
        image_input = preprocess(image).unsqueeze(0).to(device)
        text_inputs = torch.cat([clip.tokenize(f"a photo of a {c}") for c in cifar100.classes]).to(device)  
        outputs_gender = model(**inputs_gender)
        logits_per_image = outputs_gender.logits_per_image
        probs = logits_per_image.softmax(dim=1)
        predicted_class_idx = probs.argmax(-1).item()
        print("--------------------------------------------")
        print("class:",self.label_gender[predicted_class_idx])
        print("score:", probs)
        
        # Print the result
        print("\nTop predictions:\n")
        for value, index in zip(values, indices):
            print(f"{cifar100.classes[index]:>16s}: {100 * value.item():.2f}%")
    
    def executeBinaryClassification(self, area, class_list):
        pass
    
    # 前提条件：
    # 椅子の個数も変わる, 1人座ってる
    # 中心に椅子群があるとは限らない(おさまって入ることを想定)
    def executeDetectSittinChair(self, chiar_num):
        prompt = "目の前にある{chair_num}個の椅子で、人が座っている椅子は"
        
        
        
print("Label probs: ", probs)