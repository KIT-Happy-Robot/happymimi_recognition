# recognition_processing
## Overview
darknet yoloから得られた物体の認識・検出結果を用いたモジュールをまとめたパッケージ  


## Description
このパッケージが提供する機能は以下の4つです。
- Find ： 物体の探索
- List ： 検出した物体の一覧を取得
- Count ： 物体を数える
- Localize ： 物体の三次元位置の推定
  
※"物体"という言葉を使っていますが、把持可能物体だけでなく、人やtvモニターなどdarknet yoloで定義したカテゴリが対象です。

## Usage
### Find
その場で正面から+-45°の範囲で指定された物体を探すモジュール
入力されたデータに合わせて物体を探す。一定時間内に見つからなければFalseを返す。  
  
**仕様**
|Module|Communication|Name|Type|Request|Result|
|:---|:---|:---|:---|:---|:---|
|Find|Service|/recognition/find|[RecognitionFind](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionFind.srv)|string型: `target_name`|bool型: `result`|
  
### List
YOLOで検出した物体の一覧を取得する。ソートオプションを指定することで、左・中央・右を起点としてソートすることができるモジュール
  
**仕様**
|Module|Communication|Name|Type|Request|Result|
|:---|:---|:---|:---|:---|:---|
|List|Service|/recognition/list|[RecognitionList](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionList.srv)|string型: `target_name`<br>string型: `sort_option`|string[]型: `object_list`|
  
### Count
指定した物体を数えるモジュール
入力されたデータに合わせて個数を数える。  
  
**仕様**
|Module|Communication|Name|Type|Request|Result|
|:---|:---|:---|:---|:---|:---|
|Count|Service|/recognition/count|[RecognitionCount](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionCount.srv)|string型: `target_name`|int64型: `num`|
  
### Localize
指定した物体のロボット座標系における三次元位置を推定するモジュール  
物体の座標（ロボット座標系）を取得するモジュール  
  
**仕様**
|Module|Communication|Name|Type|Request|Result|
|:---|:---|:---|:---|:---|:---|
|Localize|Service|/recognition/localize|[RecognitionLocalize](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionLocalize.srv)|string型: `target_name`<br>[happymimi_msgs/StrInt型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/msg/StrInt.msg): `sort_option`|geometry_msgs/Point型: `centroid_point`|
