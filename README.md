# happymimi_recognition
## Overview
認識処理のプログラムや、それに付随するプログラム関係をまとめたメタパッケージ

## Description
以下のパッケージを含みます。
- ### [recognition_processing](./recognition_processing)
    > darknet yoloから得られた物体の認識・検出結果を用いたモジュールをまとめたパッケージ  

    このパッケージでできること
    - 周辺の物体を探す
    - YOLOで検出した物体の一覧を取得
    - 物体を数える
    - 物体の三次元位置を推定する
    - etc..

- ### [three_dimensional_data_processing](./three_dimensional_data_processing)
    > recognition_processingなどで必要となる、3次元データ処理を行うパッケージ  
    
    このパッケージでできること
    - 画像のピクセル座標から三次元位置を推定する
    - etc..
    
以上、認識系モジュールを扱うパッケージとなっています。  

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

## Bring Up
基本以下の２つを立ち上げてください  
- recognition_processingの立ち上げ
```
$ roslaunch recognition_processing recognition_processing.launch
```
- three_dimensional_data_processingの立ち上げ
```
$ roslaunch three_dimensional_data_processing three_dimensional_data_processing.launch
```

## Usage
モジュールの呼び出し方法等は各種パッケージで記述

## Reverse Lookup
- 物体を探したい 👉 [recognition_processingの Find モジュール]()
- YOLOで検出した物体の一覧が欲しい 👉 [recognition_processingのListモジュール]()
- 物体の数を知りたい 👉 [recognition_processingのCountモジュール]()
- 物体の三次元位置を知りたい 👉 [recognition_processingのLocalizeモジュール]()
- (開発向け)物体の三次元位置を知りたい 👉 [three_dimensional_data_processingのPosition Estimatorモジュール]()
- (Advanced)認識から把持の一連のタスクを行いたい 👉 [happymimi_manipulation](https://github.com/KIT-Happy-Robot/happymimi_manipulation)
