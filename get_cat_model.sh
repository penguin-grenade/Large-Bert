#!/bin/bash
# Downloads the MobileNet-SSD v2 (COCO) detector used by cat_patrol.py (~70 MB).
set -e
cd "$(dirname "$0")"
mkdir -p cat_model
cd cat_model
if [ ! -f frozen_inference_graph.pb ]; then
    wget -q --show-progress http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_coco_2018_03_29.tar.gz
    tar -xzf ssd_mobilenet_v2_coco_2018_03_29.tar.gz
    mv ssd_mobilenet_v2_coco_2018_03_29/frozen_inference_graph.pb .
    rm -rf ssd_mobilenet_v2_coco_2018_03_29 ssd_mobilenet_v2_coco_2018_03_29.tar.gz
fi
if [ ! -f ssd_mobilenet_v2_coco_2018_03_29.pbtxt ]; then
    wget -q --show-progress https://raw.githubusercontent.com/opencv/opencv_extra/4.x/testdata/dnn/ssd_mobilenet_v2_coco_2018_03_29.pbtxt
fi
mkdir -p ../cats/intruder ../cats/mine
echo "Model ready in $(pwd)"
