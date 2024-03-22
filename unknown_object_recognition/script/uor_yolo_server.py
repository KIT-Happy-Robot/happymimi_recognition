#!/usr/bin/env python3

# Desc:

import sys
import time
import rospy
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
from ultralytics_ros import YoloResult
from happymimi_msgs.srv import StrTrg # for Finding
from happymimi_recognition_msgs import UorYolo, UorYoloResponse
import roslib
base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
sys.path.insert(0, base_path)
import label_tmp
from image_module import ImageModule
import numpy as np


# IN: Sub(Yolo-world Results /uor/yolo_result) | OUT: Server (detecttion and localizeation, finding)
class UnknownObjectYoloServer():
    def __init__(self):
        rospy.init_node("uor_yolo_server")
        rospy.loginfo("Initializing Node: ** uor_yolo_server **")
        
        rospy.Subscriber("/uor/yolo_result", YoloResult, self.yoloCB)
        # IN: classes | OUT: bbox(Pose2D center[x,y,theta], size_x 0.0, size_y 0.0)
        self.yolo_ss = rospy.Service("/uor/yolo_server", UorYolo, self.serviceCB)
        self.find_ss = rospy.Service("/uor/yolo_server/finding", StrTrg, self.findServiceCB)# cla
        # 物体分類サーバー　IN: classes[], camera_name, area | OUT: class_name, item_category
        #self.clasify_ss = rospy.Service("/uor/yolo_server/clasifition", UorYolo, self.clasifyCB)
        # 物体検出、検知サーバー IN: Classes| OUT: results["obj":[xyz], conf,,,]
        self.detecttions_ss = self.rospy.Service("/uor/yolo_server/detections", UorYolo, self.detectService)
        self.IM = ImageModule()
        self.IM.rosInit(head_depth=True)
        rospy.loginfo("UnknownObjectYoloServer: Im Ready to response...")
        
    # subs
    def yoloCB(self, bb):
        self.update_time = time.time()
        self.update_flg = True
        self.bbox = bb
    def initializeBbox(self):
        if time.time() - self.update_time > 1.0 and self.update_flg:
            self.bbox = []
            self.update_flg = False
            rospy.loginfo('initialize')
    def getBboxLists(self,bb):
        bbox_list = []
        for i in bb.detections.detections:
            for j in range(len(i.results)):
                obj = i.results[j].id
                obj_name = self.object_id[str(obj)]
                bbox_list.append(obj_name)
        return bbox_list # 

    # 推論するクラスリストについて、それぞれクラスIDを順に割り振る
    def getResultClassName(self, class_id):###
        class_ids = self.class_id_list
    
    # for Yolo Service
    def applyFormat(self, results):
        # formatted_results = []
        # for result in results[0]:
        #     formatted_result = {
        #         "class_id": self.class_id_to_name[result['class']],  # クラスIDからクラス名に変換
        #         "bbox": result['bbox'],  # バウンディングボックスの座標
        #         "score": result['conf'],  # 検出の信頼度
        #         "prob": result['class_prob']  # クラス確率
        #     }
        #     formatted_results.append(formatted_result)
        # return formatted_results
        for result in results:
            boxes = result[0].boxes  # Boxes object for bbox outputs
            masks = result[0].masks  # Masks object for segmentation masks outputs
            probs = result[0].probs  # Class probabilities for classification outputs
        results_list = []
        # resultsから検出した結果を順番に処理する
        for result in results:
            result_list = {
                'boxes': result.boxes,
                'probs': result.probs,
            }
            results_list.append(result_list)
        self.resuts_list = result_list
        return result_list
        # 物体検出結果からラベル名のリストを取得
        #detected_labels = [obj.label for obj in yolo_result.objects]
    
    # 生のリザルトなので要整形
    def getPredictResults(self, classes, image):
        self.model.set_classes(classes)
        results = self.model.predict(image, 
                                     device=self.device,
                                     save=True)
        # Show results
        results[0].show()
        output_img = results[0].plot()
        output_img = self.bridge.cv2_to_imgmsg(results[0], encoding="bgr8")
        #return output_img
        #results_lists = self.getResultList(results)
        return results

    # service ------------------------------
    def serviceCB(self, yolo_msg):
        yolo_result = self.getPredictResults(msg)
        
    def getFindingResult(self, results): ###
        pass
    def findServiceCB(self, req):
        result = StrTrg()
        
        return result

    # IN: Classes| OUT: results["class_name":[xyz], conf,,,]
    def detectServiceCB(self, req): pass
    # IN: Classes | OUT: 
    #def multipleLocalize(): pass
    
    

if __name__ = "__main__"
    try:
        UOYS = UnknownObjectYoloServer()
        rospy.spin()
    except:
        pass
self.applyFormat(results)
