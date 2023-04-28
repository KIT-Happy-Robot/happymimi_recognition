# person_feature_extraction
## Overview
認識処理系を利用した人の特徴を取得するパッケージ  


## Description
このパッケージが提供する機能は以下の4つです。
- [Height_Estimation](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#height_estimation) ： 身長の推定

## Usage
### GPT
RealSenseから取得した画像データから得られた情報をCLIPで処理するモジュール  

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/gpt | [happymimi_recognition_msgs/Clip型](https://github.com/KIT-Happy-Robot/happymimi_robot/blob/develop/happymimi_msgs/srv/StrToStr.srv) | str型：`data` | str型: `result` |
  
推定に失敗した場合はエラーを返します。  
  
---  
