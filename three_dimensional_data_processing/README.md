# three_dimensional_data_processing
## Overview
recognition_processingなどで必要となる、3次元データ処理を行うパッケージ  
  
---
## Usage
ピクセルの座標を入力するとその深度を返すモジュール  
  
**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | `/detect/depth` | [PositionEstimator](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/PositionEstimator.srv) | int64型: `center_x`<br>int64型: `center_y` | geometry_msgs/Point型: `point` |
  
**center_x, center_yについて**  
RealSenseだと画角が480×640なので、、center_xは480の方向、center_yは640の方向になります。
  
---
