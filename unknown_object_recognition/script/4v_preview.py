#!/usr/bin/env python3

# import os
# import yaml
# from pathlib import Path
# from PIL import Image
# api_key_in = os.environ.get("OPEN_AI_KEY", "default")
# from openai import OpenAI
# # openai.api_key=
# client = OpenAI(api_key=api_key_in)

# # 前提条件：
# # 椅子の個数も変わる, 1人座ってる
# # 中心に椅子群があるとは限らない(おさまって入ることを想定)
# #def executeDetectSittinChair(self, chiar_num):
# #    prompt = "目の前にある{chair_num}個の椅子で、人が座っている椅子は"

# parent_dir = Path(__file__).parent.resolve()
# img_dir = parent_dir.parent/"image"
# img_file = img_dir/"pcp_2.jpg"
# #img_dir = "./../image/"
# for filename in os.listdir(img_dir):
#     if filename.endswith('.png'):  # 画像ファイルの拡張子を指定
#         image_path = os.path.join(img_dir, filename)
#         image = Image.open(image_path) # 画像を読み込み

# # file_name = "pipe_chair_person.jpg"
# # image_file = "./../image/"+file_name
# # image = 


# # プロンプト読み込み
# # parent_dir = Path(__file__).parent.resolve()
# # config_dir = parent_dir.parent/"config"
# # yaml_file = str(config_dir)+"/prompt_tmp.yaml"
# # with open(yaml_file, 'r') as file:
# #     yaml_data = yaml.safe_load(file)
# # # adjective_classのキーに対応する値を取得
# # rece_tmps = yaml_data['recetionist'][2]
# # chair_num = 3
# # prompt =  rece_tmps.format(chair_num=chair_num)
# #prompt = "目の前にある{chair_num}個の椅子で、人が座っている椅子はどの椅子ですか"
# limit_sentence = ""

# prompt = "What number of chair is sitting on person, from left of this photo"#"How many chairs are there to the right of the person sitting?"
# prompt = prompt + "Output in just one word."


# response = client.chat.completions.create(
#   model="gpt-4-vision-preview",
#   messages=[
#     {
#       "role": "user",
#       "content": [
#         {"type": "text", "text": prompt}, # "What’s in this image?"
#         {
#           "type": "image_url",
#           "image_url": {
#             "url": "https://t16.pimg.jp/016/248/846/1/16248846.jpg",#image,# image, # "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg",
#             # https://cpw.imagenavi.jp/preview/163/16322514_PW36.jpg
#             # https://t18.pimg.jp/016/248/828/1/16248828.jpg # 3(4)
#             # https://t16.pimg.jp/016/248/846/1/16248846.jpg # 3 (4)
#           },
#         },
#       ],
#     }
#   ],
#   max_tokens=100,
# )

# print(response.choices[0].message.content)

# ----
import os
import yaml
import json
from pathlib import Path
from PIL import Image
api_key_in = os.environ.get("OPEN_AI_KEY", "default")
from openai import OpenAI
# openai.api_key=
client = OpenAI(api_key=api_key_in)
import base64
import requests
parent_dir = Path(__file__).parent.resolve()
img_dir = parent_dir.parent/"image"
img_file = img_dir/"pcp_2.png"
#img_dir = "./../image/"
for filename in os.listdir(img_dir):
    if filename.endswith('.png'):  # 画像ファイルの拡張子を指定
        image_path = os.path.join(img_dir, filename)
        image = Image.open(image_path) # 画像を読み込み
# Function to encode the image
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')
#image_path = "path_to_your_image.jpg"
# Getting the base64 string
base64_image = encode_image(img_file)
headers = {
  "Content-Type": "application/json",
  "Authorization": f"Bearer {api_key_in}"
}
payload = {
  "model": "gpt-4-vision-preview",
  "messages": [
    {
      "role": "user",
      "content": [
        {
          "type": "text",
          "text": "What’s in this image?"
        },
        {
          "type": "image_url",
          "image_url": {
            "url": f"data:image/jpeg;base64,{base64_image}"
          }
        }
      ]
    }
  ],
  "max_tokens": 300
}
response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
print(response.json())

print(response.json()["choices"][0]["message"]["content"])
