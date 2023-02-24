# recognition_processing
## Overview
Darknet YOLOから得られた物体の認識・検出結果を用いたモジュールをまとめたパッケージ  

このパッケージが提供する機能は以下の4つです。
- [Save](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#save) : 認識画像の保存
- [Find](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#find) ： 物体の探索
- [List](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#list) ： 検出した物体の一覧を取得
- [Count](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#count) ： 物体を数える
- [Localize](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#localize) ： 物体の三次元位置の推定
- [Multiple_Localize](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/recognition_processing#multiple_localize) ： 複数物体の三次元位置の推定

※"物体"という言葉を使っていますが、把持可能物体(ex. cupとかbottleとか)だけでなく、人やtvモニターなどDarknet YOLOで定義したカテゴリが対象です。

## Usage
### Save
認識結果であるboundin boxを描画した画像を保存するモジュール  
リクエストで保存するパスを指定できる  
保存される画像の名前は(時間).pngになります  

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition/save | [StrTrg](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/StrTrg.srv) | string型: `data` | bool型: `result` |

dataの例  
`'/home/mimi/recognition'`  

---

### Find
その場で正面から+-45°の範囲で指定された物体を探すモジュール  
入力されたデータに合わせて物体を探す。一定時間内に見つからなければFalseを返す。  

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition/find | [RecognitionFind](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionFind.srv) | string型: `target_name` | bool型: `result` |

**target_nameの種類**
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. personとかcupとか) | 指定した名前の物体を探す |
| any | 把持可能物体を探す |
| (入力なし) | 何かしらの物体を探す |

---
### List
YOLOで検出した物体の一覧を取得するモジュール  
ソートオプションを指定することで、左・中央・右を起点としてソートすることができる。

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition/list | [RecognitionList](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionList.srv) | string型: `target_name`<br>string型: `sort_option` | string[]型: `object_list` |

**target_nameの種類**
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. personとかcupとか) | 指定した名前の物体のみのリストを返す |
| any | 把持可能物体のみのリストを返す |
| (入力なし) | YOLOで検出した全ての物体のリストを返す |

**sort_optionの種類（任意）**
| sort_option | Contents |
| :---: | --- |
| left | 画面左から順に並び替えたリストを返す |
| center | 画面中央から順に並び替えたリストを返す |
| right | 画面右から順に並び替えたリストを返す |

---
### Count
指定した物体を数えるモジュール  
入力されたデータに合わせて物体の個数を数える。  

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition/count | [RecognitionCount](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionCount.srv) | string型: `target_name` | int64型: `num` |

**target_nameの種類**
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. personとかcupとか) | 指定した名前の物体の数を返す |
| any | 把持可能物体の数を返す |

---
### Localize
指定した物体のロボット座標系における三次元位置を推定するモジュール  
ソートオプションを指定することで、左・中央・右を起点とした順番に物体を並び替え、取得したい物体を指定することができる。

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition/localize | [RecognitionLocalize](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/RecognitionLocalize.srv) | string型: `target_name`<br>[happymimi_msgs/StrInt型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/msg/StrInt.msg): `sort_option` | geometry_msgs/Point型: `point` |

**target_nameの種類**
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. personとかcupとか) | 一つの指定した名前の物体の三次元位置を返す |
| any | 一つの把持可能物体の三次元位置を返す |

**sort_optionの種類（任意）**
| data | Contents |
| :---: | --- |
| left | 画面左から順に並び替えたリストを返す |
| center | 画面中央から順に並び替えたリストを返す |
| right | 画面右から順に並び替えたリストを返す |

| num | Contents |
| :---: | --- |
| 数値 | ソートしたリストを元に、指定した数値番目の物体の三次元位置を取得 |

---
### Multiple_Localize
指定した名前に該当するすべての物体の三次元位置をリストで返すモジュール  

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /recognition/multiple_localize | [MultipleLocalize](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/MultipleLocalize.srv) | string型: `target_name` | geometry_msgs/Point[]型: `points` |

**target_nameの種類**
| target_name | Contents |
| :---: | :--- |
| 特定の名前(ex. personとかcupとか) | 指定した名前に該当するすべての物体の三次元位置を返す |
| any | すべての把持可能物体の三次元位置を返す |

---
