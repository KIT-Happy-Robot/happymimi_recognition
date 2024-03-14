#!/usr/bin/env python3

import os
import yaml
api_key_in = os.environ.get("OPEN_AI_KEY", "default")
from openai import OpenAI
# openai.api_key=
client = OpenAI(api_key=api_key_in)

# 前提条件：
# 椅子の個数も変わる, 1人座ってる
# 中心に椅子群があるとは限らない(おさまって入ることを想定)
#def executeDetectSittinChair(self, chiar_num):
#    prompt = "目の前にある{chair_num}個の椅子で、人が座っている椅子は"
img_dir = "./../image/"
for filename in os.listdir(img_dir):
    if filename.endswith('.jpg'):  # 画像ファイルの拡張子を指定
        image_path = os.path.join(img_dir, filename)
        image = Image.open(image_path) # 画像を読み込み

# file_name = "pipe_chair_person.jpg"
# image_file = "./../image/"+file_name
# image = 

# プロンプト読み込み
yaml_file = "./../config/prompt_tmp.yaml"
with open(yaml_file, 'r') as file:
    yaml_data = yaml.safe_load(file)
# adjective_classのキーに対応する値を取得
adjective_class_list = yaml_data['recetionist']

prompt = "目の前にある{chair_num}個の椅子で、人が座っている椅子はどの椅子ですか"
limit_sentence = ""


response = client.chat.completions.create(
  model="gpt-4-vision-preview",
  messages=[
    {
      "role": "user",
      "content": [
        {"type": "text", "text": prompt}, # "What’s in this image?"
        {
          "type": "image_url",
          "image_url": {
            "url": image, # "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg",
          },
        },
      ],
    }
  ],
  max_tokens=100,
)

print(response.choices[0].message.content)
