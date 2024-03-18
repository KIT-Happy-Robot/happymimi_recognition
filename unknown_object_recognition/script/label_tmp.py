#!/usr/bin/env python3 

# Yamlだと代入とかできなくて不便だからPythonスクリプトのも作っておく

default_classes = [
    "cup",
    "bottle",
    "box",
    "chair",
    "table",
]

tidyup_classes=[
    "white_cup", # noodle
    "green_box", # "treat_box", "long_block"
    "orange cup", #"potato chips"
    "yellow_bottle", # "detergent"
    "blue_cup", # "cup"
    "red_case", #lunch box ### cup, case, round case
    "white_block", #dice ### cup, white block
    "black_box", #light bulb box, black block###
    "block box" # treat box, box, block###
]

cifar10_classes = [
    'airplane',
    'automobile',
    'bird',
    'cat',
    'deer',
    'dog',
    'frog',
    'horse',
    'ship',
    'truck',
]

cifar10_templates = [
    'a photo of a {}.',
    'a blurry photo of a {}.',
    'a black and white photo of a {}.',
    'a low contrast photo of a {}.',
    'a high contrast photo of a {}.',
    'a bad photo of a {}.',
    'a good photo of a {}.',
    'a photo of a small {}.',
    'a photo of a big {}.',
    'a photo of the {}.',
    'a blurry photo of the {}.',
    'a black and white photo of the {}.',
    'a low contrast photo of the {}.',
    'a high contrast photo of the {}.',
    'a bad photo of the {}.',
    'a good photo of the {}.',
    'a photo of the small {}.',
    'a photo of the big {}.',
]
