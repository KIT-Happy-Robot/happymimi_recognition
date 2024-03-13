#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import torch
import clip
import numpy as np
from sklearn.cluster import DBSCAN

# CLIPモデルをロード
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# コールバック関数
def callback(img_msg, pcl_msg):
    # 画像とポイントクラウドデータを取得
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
    pcl = ros_pcl.pointcloud2_to_xyz_array(pcl_msg)

    # 画像から物体のbounding boxを取得
    boxes = get_bboxes(img)

    # 各bounding boxについて処理
    for box in boxes:
        # bounding boxからポイントクラウドデータを取得
        obj_pcl = get_pcl_from_bbox(pcl, box)

        # CLIPでクラス推定
        img_input = preprocess(Image.fromarray(obj_pcl)).unsqueeze(0).to(device)
        text_inputs = torch.cat([clip.tokenize(f"a photo of a {c}") for c in classes]).to(device)
        with torch.no_grad():
            image_features = model.encode_image(img_input)
            text_features = model.encode_text(text_inputs)
            logits_per_image, logits_per_text = model(image_features, text_features)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        # 最も確率の高いクラスを取得
        obj_class = classes[probs.argmax()]
        print(f"Detected object: {obj_class}")

        # 分類結果をパブリッシュ
        publish_result(obj_class, box)

# メイン関数
def main():
    rospy.init_node('object_classifier')
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=1)
    pcl_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()