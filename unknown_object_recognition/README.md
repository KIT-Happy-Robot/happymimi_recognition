# Unknown Object Recognition 
実装したYoloなどの物体検出ルゴリズムで検出できかった未知物体の認識を実現させるためのパッケージ、通称「Unkパッケージ」です。
RCJ2024の見極めまでに時間がなくて、ここ4日で1人ですべてモジュールを作っていたのでコミット文をちゃんと書いてません。
ご了承ください。

# Overview
- src
  cppファイルの格納庫
- script
  今の所はpythonスクリプトの格納庫
  -　table_object_server.py: RSカメラの点群データから目の前のテーブル、平面の上にある何かしらの物体の情報を推定して返すサービスサーバー
  - pc_module: 上のスクリプトや今後点群を用いた機械学習モデルとの等号に利用するものをまとめるスクリプトファイル
  - uor_yolo_server.py: 指定した何かしらの物体の名前のリストを受け取り、検出できた分の物体の情報だけ返すサービスサーバー
  - label_temp.py: Yamlファイルが不便だったので、Pythonで上のスクリプトに利用するリストや辞書をまとめるスクリプトファイル
  - uor_gpt_server: 物体を含んだローカル画像データとプロンプトを投げて帰ってきた結果をクライアントへ返すサービスサーバー
  - uor_clip_server: 上に同じく。
  - prompt_modele.py: Yamlファイルが不便だったので、Pythonで上2つのスクリプトに利用するプロンプトをまとめるスクリプトファイル
  - image_server.py: 指定したカメラ（おでこ、アーム先）・フォーマット（ros,cv,jpeg,png）・画角範囲の画像を返すサービス
  - image_module.py: 上の処理や画像取得、処理、変換などを行う関数セットをまとめたスクリプトファイル
- config
  動かす学習モデルや重みの設定、物体クラス名、プロンプトなどの構成ファイルの格納庫
  物体用の一時座標辞書も含む予定
- launch
- image
  テスト画像や取得した画像、推論結果画像などの格納庫
- model
  今はYoloの重みファイルの格納庫

# Installtion
```
git clone https://github.com/KIT-Happy-Robot/happymimi_recognition
cd ~
# Install YOLO-World　-----------------------------------
git clone --recursive https://github.com/AILab-CVC/YOLO-World.git
pip3 install torch wheel -q
pip3 install -e
# Hugging faceのやつの場合(HFとGitのとで重みの性能など違う点があるので要確認。鷲尾は自分のPCではこっちで最初やった)
git clone https://huggingface.co/spacs/stevengrove/YOLO-World
pip3 install -r requirements.txt
pip3 install -U ultralytics
# Proxy配下用
pip3 install --proxy http://wwwproxy.kanazawa-it.ac.jp:8080 -r requirements.txt
pip3 install --proxy http://wwwproxy.kanazawa-it.ac.jp:8080 -U ultralytics
# Install CLIP ---------------------------------------------
# CLIPにも２つほどあるっぽいがIssues（https://github.com/openai/CLIP/issues/180）を参考にこれでやった。
pip3 install openai-clip
# Proxy配下用
pip3 install openai-clip --proxy http://wwwproxy.kanazawa-it.ac.jp:8080
# Transformerも一応
pip3 install transformers
# GPT ------------------------------------------------------
pip3 install openai
# BLIP（未実装） ---------------------------------------------
pip3 install requests pillow transformers
```

# Usage

# ROS Usage

# example
## Yolow Results
[Yolow Results Image 1](https://github.com/KIT-Happy-Robot/happymimi_recognition/edit/master/unknown_object_recognition/image/dspt_tu_2.jpg)
[Yolow Results Image 2](https://github.com/KIT-Happy-Robot/happymimi_recognition/edit/master/unknown_object_recognition/image/up_real_tu_1.jpg)
