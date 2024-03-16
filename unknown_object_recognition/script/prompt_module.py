#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import roslib
import rosparam

class Prompt():
    def __init__(self):
        #lists = rospy.get_param('objects_list', [])
        self.loadLists()
        self.setMultipleClasification()
        
    def loadLists(self):
        # Tidyup用のオブジェクトリスト
        # with open(pkg_dir/"config/object_list.yaml", 'r') as file:
        #     self.object_dict = yaml.safe_load(file)
        with open(pkg_dir/"config/item_category.yaml", 'r') as file:
            self.object_dict = yaml.safe_load(file)
        # self.tidyup_object_dict = rospy.get_param('item_category')
        self.tidyup_category_list = list(self.tidyup_object_dict.keys()) # (# dict_keys(['',,,]))
        self.tidyup_item_list = [value for values in self.tidyup_object_dict.values() for value in values]

    def setMultipleClasification(self):
        self.classify_center_tidyup_object_name = (
            f"Guess the name of the object in the center of this image in object list: {self.tidyup_object_list}")
        self.classify_tidyup_object_category = (
            f"Guess the name of the object category in this image in object list: {self.tidyup_object_list}")