#!/usr/bin/env python3

"""
ROS server: return data from results of yolo-world zeroshot prediction.
IN: classes(include class list name), camera name,,, | OUT: ids, names, bbox sizes, center coords
Requirements:
    pip3 install ultralytics
    ...
References:
    - https://docs.ultralytics.com/models/yolo-world/#key-features
    - https://docs.ultralytics.com/#where-to-start
    - https://docs.ultralytics.com/ja/reference/engine/results/
Author: washio
"""

import sys
import yaml
import time
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLOWorld
from ultralytics_ros.msg import YoloResult
from happymimi_msgs.srv import StrTrg # for Finding
from happymimi_recognition_msgs.srv import UorYolo, UorYoloResponse, CameraImage, CameraImageRequest
from pathlib import Path
# import roslib
# base_path = roslib.packages.get_pkg_dir('unknown_object_recognition') + '/script/'
# sys.path.insert(0, base_path)

# IN: Sub(Yolo-world Results /uor/yolo_result) | OUT: Server (detecttion and localizeation, finding)
class UnknownObjectYoloServer():
    def __init__(self):
        rospy.init_node("uor_yolo_server")
        rospy.loginfo("Initializing Node: ** uor_yolo_server **")
        rospy.Subscriber("/uor/yolo_result", YoloResult, self.yoloCB)
        rospy.Subscriber("/uor/yolo_test", Empty, self.test)
        #rospy.wait_for_service("/recognition/image_server")
        self.image_sc = rospy.ServiceProxy("/recognition/image_server", CameraImage)
        # IN: classes | OUT: bbox(Pose2D center[x,y,theta], size_x 0.0, size_y 0.0)
        self.yolo_ss = rospy.Service("/uor/yolow_server", UorYolo, self.serviceCB)
        #self.find_ss = rospy.Service("/uor/yolo_server/finding", StrTrg, self.findServiceCB)# cla
        # 物体分類サーバー　IN: classes[], camera_name, area | OUT: class_name, item_category
        #self.clasify_ss = rospy.Service("/uor/yolo_server/clasifition", UorYolo, self.clasifyCB)
        # 物体検出、検知サーバー IN: Classes| OUT: results["obj":[xyz], conf,,,]
        #self.detecttions_ss = self.rospy.Service("/uor/yolo_server/detections", UorYolo, self.detectService)
        print("ros tool get ready...")
        self.bridge = CvBridge()

        self.model_config = rospy.get_param('/uor/model_config', {})
        self.model_name = self.model_config["yolo"]["model"]
        # load
        print("Loading model ...")
        self.model = YOLOWorld(self.model_name)
        self.conf_thres = self.model_config["yolo"]["conf_thres"]
        self.device = self.model_config["device"]
        pkg_dir = Path(__file__).parent.resolve().parent
        self.uor_model_config = rospy.get_param("/uor/object_class_list")
        with open(pkg_dir/"config/object_class_list.yaml", 'r') as file:
            self.object_class_list = yaml.safe_load(file)
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
                                    conf=self.conf_thres,
                                    save=True)
        # Show results
        results[0].show()
        output_img = results[0].plot()
        output_img = self.bridge.cv2_to_imgmsg(results[0], encoding="bgr8")
        #return output_img
        #results_lists = self.getResultList(results)
        return results
    
    def createDetectionsArray(self, results):
        detections_msg = Detection2DArray()
        bbox_list = []
        bbox_data = {}
        self.ids = []
        self.names = []
        self.center_xs =[]
        self.center_ys =[]
        self.size_xs = []
        self.size_ys = []
        self.points = []
        bounding_box = results[0].boxes.xywh
        classes = results[0].boxes.cls
        names = results[0].names
        confidence_score = results[0].boxes.conf
        for bbox, cls, name, conf in zip(bounding_box, classes, names, confidence_score):
            detection = Detection2D()
            detection.bbox.center.x = float(bbox[0])
            detection.bbox.center.y = float(bbox[1])
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)
            hypothesis.score = float(conf)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)
            self.points.append([int(bbox[0]), int(bbox[1])])
            bbox_dict = {'id': int(cls), 'name': str(name), 
                        'center': [bbox[0], bbox[1]], 'size': [bbox[2], bbox[3]]}
            bbox_list.append(bbox_dict)
            self.ids.append(int(cls)); 
            self.names.append(str(names))
            self.center_xs.append([bbox[0]])
            self.center_ys.append([bbox[1]])
            self.size_xs.append([bbox[2]])
            self.size_xs.append([bbox[3]])

        return detections_msg, bbox_list
    
    def setModelClasses(self, classes): self.model.set_classes(classes)
    # service ------------------------------
    def serviceCB(self, req):
        UY = UorYoloResponse()
        if req.camera_name == "head" or req.camera_name == None:
            image = self.image_sc("head",  req.depth_musk).image #cv_img = IM.getHeadCvImage()
        if req.camera_name == "arm":
            image = self.image_sc("arm",  req.depth_musk).image #cv_img = IM.getArmCvImage()
        if req.camera_name == "usbcam":
            image = self.image_sc("usbcam",  req.depth_musk).image # cv_img = IM.getUsbCvImage()
        #UY = UorYoloResponse()
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        if req.classes:
            if req.classes[0] == "tidyup_fix": self.setModelClasses(self.object_class_list["tidyup_fix"])
            elif req.classes[0] == "yumeko_tu": self.setModelClasses(self.object_class_list["yumeko_tu"])
            else: self.setModelClasses(req.classes)
        else: self.setModelClasses(self.object_class_list["default"])
        results = self.model.predict(source=cv_image,
                                    conf=self.conf_thres,
                                    save=True)
        detect_msg, bbox_list = self.createDetectionsArray(results)
        for data in bbox_list:
            UY.id.append(data['id'])
            UY.name.append(data['name'])
            UY.center_x.append(int(data["center"][0]))
            UY.center_y.append(int(data["center"][1]))
            UY.size_x.append(int(data['size'][0]))
            UY.size_y.append(int(data['size'][1]))
        return UY
        
    def test(self, _):
        #image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        image = self.image_sc("head",  False).image
        image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        self.setModelClasses(["cup", "box"])
        results = self.model.predict(source=image,
                                    conf=self.conf_thres)
        results[0].show
        detect_msg, bbox_list = self.createDetectionsArray(results)
        id=[]
        name=[]
        center_x=[]
        center_y=[] 
        size_x=[]
        size_y = []
        for data in bbox_list:
            id.append(data['id'])
            name.append(data['name'])
            center_x.append(data["center"][0])
            center_y.append(data["center"][1])
            size_x.append(data['size'][0])
            size_y.append(data['size'][1])
        print(id)
        print(name)
        print(center_x, center_y, size_x, size_y)
        return True
    def getFinding(self, name): ###
        pass

    # IN: Classes| OUT: results["class_name":[xyz], conf,,,]
    def detectServiceCB(self, req): pass
    # IN: Classes | OUT: 
    #def multipleLocalize(): pass
    
    

if __name__ == "__main__":
    try:
        UOYS = UnknownObjectYoloServer()
        rospy.spin()
    except:
        pass

