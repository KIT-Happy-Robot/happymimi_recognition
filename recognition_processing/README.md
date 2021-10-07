# recognition_processing
## Overview
darknet yoloから得られた物体の認識・検出結果を用いたモジュールをまとめたパッケージ

## Description
このパッケージでできること
- **Find ： 物体の探索**  
    その場で正面から+-45°の範囲で指定された物体を探すモジュール
- **List ： 検出した物体の一覧を取得**    
    yoloで検出した物体の一覧を取得する。ソートオプションを指定することで、左・中央・右を起点としてソートすることができるモジュール
- **Count ： 物体の数を数える**  
    指定した物体を数えるモジュール
- **Localize ： 物体の三次元位置の推定**  
    指定した物体のロボット座標系における三次元位置を推定するモジュール  
  
※"物体"という言葉を使っていますが、把持可能物体だけでなく、人やtvモニターなどdarknet yoloで定義したカテゴリが対象です。

## Usage
### recognition_processingの立ち上げ
```
$ roslaunch recognition_processing recognition_processing.launch
```

### モジュールの使い方について
  |Module|Communication|Name|Type|Request|Result|  
  |:---|:---|:---|:---|:---|:---|  
  |Find|Service|`'/recognition/find'`|[RecognitionFind]()|string型の`target_name`|bool型の`result`|  
  |List|Service|/recognition/list|[RecognitionList]()|string型の`target_name`, happymimi_msgs/StrInt型の`sort_option`|string[]型の`object_list`|
  |Count|Service|/recognition/count|[RecognitionCount]()|string型の`target_name`|int64型の`num`|  
  |Localize|Service|/recognition/localize|[RecognitionLocalize]()|string型の`target_name`|geometry_msgs/Point型の`centroid_point`|  
  
---
### 機能の詳細
### Find
物体を見つけるモジュール  
入力されたデータに合わせて物体を探す。一定時間内に見つからなければFalseを返す。  
  
機能紹介  
  
### List
検出した物体の一覧を返すモジュール  
  
機能紹介  
  
### Count
物体を数えるモジュール  
入力されたデータに合わせて個数を数える。  
  
機能紹介  
  
### Localize
物体の座標（ロボット座標系）を取得するモジュール  
  
機能紹介  
- 入力：`"物体の名前"`　の場合はその物体の座標を取得する
