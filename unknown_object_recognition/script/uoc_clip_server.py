#!/usr/bin/env python3

# CLIPで
# IN: task_name, target(obj, person, env), target_area(center, all,,,), class_list[], image
# OUT:clacified_name

import rospy
from happymimi_recognition_msgs import UorClip
from uor_module import ImageHub, ClipHub

class UnknownObjectClipServer():
    def __init__(self):
        rospy.loginfo("Initializing Node: ** Unknown object Clip Server **")
        rospy.Service("/uro/clip_server", UorClip, self.serviceCB)
        IMAGEHUB = ImageHub()
        CLIPHUB = ClipHub()
        self.object_class_list = rospy.get_param("/object_class_list", [])
    
    def serviceCV(self, req):
        image = IMAGE.get
        task_name = req.task_name
        target_name = req.target_name
        target_area = "center" if req.target_area is None else req.target_area
        class_list = self.object_class_list["default"] if req.class_list is None else req.class_list

        
        if task_name is None or task_name == "Classification":
            result = self.executeClassification(target_name, target_area, )
    
    def executeMultiClassClassification(self, ):
        prompt = 
        
    
    def executeBinaryClassification(self, area, class_list):
        pass
    