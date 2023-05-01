# person_feature_extraction
## Overview
認識処理系を利用した人の特徴を取得するパッケージ  


## Description
このパッケージが提供する機能は以下の4つです。
- [gpt2_exam_realsense](https://github.com/KIT-Happy-Robot/happymimi_recognition/tree/master/person_feature_extraction#gpt2_exam_realsense) ： 特徴の推定

## Usage
### GPT
RealSenseから取得した画像データから得られた情報をCLIPで処理するモジュール  

**仕様**
| Communication | Name | Type | Request | Result |
| :---: | :---: | :---: | :---: | :---: |
| Service | /person_feature/gpt | [happymimi_recognition_msgs/Clip型](https://github.com/KIT-Happy-Robot/happymimi_recognition/blob/master/happymimi_recognition_msgs/srv/Clip.srv) | str型：`data` | str型: `result` |
  
  
推定に失敗した場合はエラーを返します。  

---  

## CLIPって何?

- 「OpenAI CLIP」は、OpenAIが開発した、画像とテキストの関連性をランク付けするニューラルネットワーク。
- 従来の「教師あり学習」の画像分類では決められたラベルのみで分類するのに対し、「OpenAI CLIP」では推論時に自由にラベルを指定して画像分類することができるよ。

```
テキストと画像を照らし合わせて、より関連性が高いテキストを抽出する形で使用しているよ  
気になったら、このリンクに飛んでね( *´艸｀)  
```
[OpenAI CLIPの使い方](https://note.com/npaka/n/n74a9b172b41d)
