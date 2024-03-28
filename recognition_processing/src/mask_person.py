#!/usr/bin/env python3
#-*- coding: utf-8 -*

import cv2
import numpy as np
from ultralytics import YOLOWorld,YOLO

model = YOLOWorld("yolov8s-world.pt")
model.set_classes(["person"])

def mask_person(img):
    h = img.shape[0]
    w = img.shape[1]
    result = model(img)
    person = result[0].boxes.xyxy
    #print(result)
    #print(person)
    mask1 = np.zeros((h, w,3), np.uint8)
    mask2 = np.zeros((h, w,3), np.uint8)
    mask1 = cv2.rectangle(mask1, (int(person[0][0]),int(person[0][1])),(int(person[0][2]),int(person[0][3])),(255,255,255), -1)
    mask2 = cv2.rectangle(mask2, (int(person[1][0]),int(person[1][1])),(int(person[1][2]),int(person[1][3])),(255,255,255), -1)
    #detect_img = result[0].plot()
    mask1AND = cv2.bitwise_and(img,mask1)
    mask2AND = cv2.bitwise_and(img,mask2)

    return mask1AND,mask2AND

    """
    cv2.imshow("img",mask1AND)
    k = cv2.waitKey(0)
    if k == 13:
        cv2.destroyAllWindows()
    """

if __name__ == "__main__":
    image = cv2.imread("IMG_4883.jpg")
    frame1,frame2 = mask_person(image)
    cv2.imshow("frame1",frame1)
    cv2.imshow("frame2",frame2)
    cv2.waitKey(0)
    
