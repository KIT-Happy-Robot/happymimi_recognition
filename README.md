# happymimi_recognition
## Overview
認識処理のプログラムや、それに付随するプログラム関係をまとめたメタパッケージ

## Description
- ### [recognition_processing](./recognition_processing)
    darknet yoloから得られた物体の認識・検出結果を用いたモジュールをまとめたパッケージ
- ### [three_dimensional_data_processing](./three_dimensional_data_processing)
    recognition_processingなどで必要となる、3次元データ処理を行うパッケージ

## Requirement
※あまり気にしなくていいと思います
```
pcl_ros
cv_bridge
image_transport
```

## Build Environment
```
$ catkin build
```

## Usage
recognition_processingの立ち上げ
```
$ roslaunch recognition_processing recognition_processing.launch
```
three_dimensional_data_processingの立ち上げ
```
$ roslaunch three_dimensional_data_processing three_dimensional_data_processing.launch
```
