# approach_person
approach_personは人接近機能を提供するROSパッケージです。

## Description
本パッケージは以下の3つのノードから構成されています。


- ### approach_person_server
    > approach_perosn_serverは人接近を行うためのサービスサーバです。ここでは、human_coord_generatorノードにより生成された人のidと座標を用いて、人に接近する処理を行います。
    > 人のidと座標は[happymimi_params/location](https://github.com/KIT-Happy-Robot/happymimi_robot/tree/develop/happymimi_params/location)に保存された[tmp_human_location.yaml](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_params/location/tmp_human_location.yaml)を読み込むことで取得します。
    > 人の接近には[navi_locationサービスサーバ](https://github.com/KIT-Happy-Robot/happymimi_apps/blob/develop/happymimi_navigation/src/navi_location.py)を利用しており、目標が障害物として埋まってしまうのを回避するために目標位置の許容誤差を指定するパラメータ（Table1）を動的に変更しています。
    > 
    > **Table1. 目標位置の許容誤差を指定するパラメータ**
    > | パラメータ名 | 変更前 | 変更後|
    > |---|---|---|
    > | xy_goal_tolerance | 0.15 | 0.70 |
    > | yaw_goal_tolerance | 0.08 | 6.2 |
    > 
    > **サービスサーバの仕様**
    > | Name | Type | Request | Result | Note |
    > |---|---|---|---| --- |
    > | /approach_person_server | [StrTrg](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/StrTrg.srv)| string型: `data` | bool型: `result` | `data`には人のid（ex. `human_0`）を入力してください |


- ### human_coord_generator
    > human_coord_generatorは人のidと座標の辞書データを作成し、`tmp_human_location`として保存するためのサービスサーバです。
    > ここでは、[happymimi_recognition/recognition_processing](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing)の[recognition/multiple_localizeサービスサーバ](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#multiple_localize)から人のxy座標を取得し、pub_human_tfノードを用いてクォータ二オン座標に変換します。
    > さらにそれをマップ座標系に変換して人のidと紐付けることで辞書型データを作成し、[happymimi_params/location](https://github.com/KIT-Happy-Robot/happymimi_robot/tree/develop/happymimi_params/location)に`tmp_human_location`として保存します。
    > <br>また、[hapyymimi_params/location](https://github.com/KIT-Happy-Robot/happymimi_robot/tree/develop/happymimi_params/location)の`<map_name>_range.yaml`を読み込み、人の座標がマップの範囲内にあるか否かを判定する処理を行います。これは大会会場などで観客の座標を登録してしまうことを防ぎます。
    > 
    > **サービスサーバの仕様**
    > | Name | Type | Request | Result |
    > |---|---|---|---|
    > | /human_coord_generator | [SimpleTrg](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SimpleTrg.srv)| None | bool型: `result` | 


- ### pub_human_tf
    > pub_human_tfは人のクォータ二オン座標を生成するためにアクションサーバです。受け取った人のidとxy座標をもとに、クォータ二オン座標を生成し[TFMessage型](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html)のトピックとして配布します。
    > 
    > **アクションサーバの仕様**
    > | Name | Type | Goal | Result | Feedback |
    > |---|---|---|---| --- |
    > | /pub_human_tf | [PubHumanTF](https://github.com/KIT-Happy-Robot/happymimi_apps/blob/develop/approach_person/action/PubHumanTF.action)| string型: `name` <br> float32型: `dist_x` <br> float32型: `dist_y` | bool型: `data` | int8型: `state`|


<br>approach_personのデータフロー図を以下に示します。
<p align="center">
 <img src="https://user-images.githubusercontent.com/45844173/221177745-8d40892e-df6a-4850-b029-3a5becef0e6e.png" width="100%">
</p>
<p align="center">
  approach_personのデータフロー
</p>


## Usage
ロボットとセンサの起動
```
$ roslaunch happymimi_bringup minimal.launch
```
```
$ roslaunch happymimi_bringup sensor.launch
```
ナビゲーションに関するlaunchファイルの起動
```
$ roslaunch happymimi_navigation amcl.launch
```

```
$ roslaunch happymimi_rviz_launchers view_navigation.launch
```
```
$ rosrun happymimi_navigation navi_location.py
```
approach_person.launchの起動
```
$ roslaunch approach_person approach_person.launch
```
