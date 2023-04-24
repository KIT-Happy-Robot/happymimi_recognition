# person_feature_extraction
## Overview
認識処理系を利用した人の特徴を取得するパッケージ  


## Description
このパッケージが提供する機能は以下の8つです。
- [Height_Estimation](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#height_estimation) ： 身長の推定
- [Detect Clothing Color](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_clothing_color) ： 服の色（上半身）の検出
- [Detect Pants Color](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_pants_color) ： 服の色（下半身）の検出
- [Detect Hair Color](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_hair_color) ： 髪の色の検出
- [Detect Skin Color](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_skin_colorv2) ： 肌の色の検出
- [Detect Gender](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_gender) ： 性別の検出
- [Detect Glass](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_glass) ： メガネの検出
- [Detect Old](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_old) ： 年齢の推定


## Usage
### Height_Estimation
身長を推定するモジュール  
鼻の高さから大体の身長を割り出しているので精度は保証できません  
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/height | [happymimi_msgs/SetFloat型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetFloat.srv) |  | float64型: `data` |
  
推定に失敗した場合は`-1`を返します。  
  
---  
### Detect_Cloth_Color
服の色(上半身)を検出するモジュール  
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/cloth_color | [happymimi_msgs/SetStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetStr.srv) |  | str型: `result` |
  
推定に失敗した場合は`''`を返します。  
  
--- 
### Detect_Pants_Color
服の色（下半身）を検出するモジュール  
※精度が全然保証できないためあしからず(*^^*)

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/pants_color | [happymimi_msgs/SetStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetStr.srv) |  | str型: `result` |
  
推定に失敗した場合は`''`を返します。  
  
--- 
### Detect_Hair_Color
髪の色を検出するモジュール  
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/hair_color | [happymimi_msgs/SetStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetStr.srv) |  | str型: `result` |
  
推定に失敗した場合は`''`を返します。  
  
---
### Detect_skin_Colorv2
肌の色を検出するモジュール  
※RCJ2022では「このご時世では倫理的に...」とか言われてお蔵入りになった伝説の機能
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/skin_color | [happymimi_msgs/SetStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetStr.srv) |  | str型: `result` |
  
推定に失敗した場合は`''`を返します。  
  
---
Face++のAPIを使用し、特徴量を検出するモジュール
### Detect_Gender
対象者の顔から性別を判断するモジュール  
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/gender | [happymimi_msgs/SetStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetStr.srv) |  | str型: `result` |
  
推定に失敗した場合はエラーを返します。  
  
---  
### Detect_Glass
対象者の顔からメガネの有無を判断するモジュール  
※コード内ではマスクの有無も判断していますが、「今後の大会ではマスク外す方針なので...」とかいう意味不明な理由でお蔵入りになった伝説の機能

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/glass | [happymimi_msgs/StrToStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/StrToStr.srv) |  | str型: `result` |
  
推定に失敗した場合はエラーを返します。  
  
---  
### Detect_Old
対象者の顔から年齢を判断するモジュール  
※年齢の判断は「20~30代」のような範囲指定なので正確でない場合があります。 

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/old | [happymimi_msgs/StrToStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/StrToStr.srv) |  | str型: `result` |
  
推定に失敗した場合はエラーを返します。  
  
---  
