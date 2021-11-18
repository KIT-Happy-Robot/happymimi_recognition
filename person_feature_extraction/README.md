# person_feature_extraction
## Overview
認識処理系を利用した人の特徴を取得するパッケージ  


## Description
このパッケージが提供する機能は以下の2つです。
- [Height_Estimation](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#height_estimation) ： 身長の推定
- [Detect Clothing Color](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#detect_clothing_color) ： 服の色の検出
  

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
服の色を検出するモジュール  
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/cloth_color | [happymimi_msgs/SetStr型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/SetStr.srv) |  | str型: `result` |
  
推定に失敗した場合は`''`を返します。  
  
---  
