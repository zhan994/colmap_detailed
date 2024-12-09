#!/bin/sh

# 特征提取
# Zhihao Zhan

log_time() {
    date "+%Y-%m-%d %H:%M:%S:%3N"
}

PROJECT="${PWD}/proj"

python3 work/python/camera_mask.py 960 540 200 250 ${PROJECT}/camera_mask.png

echo "$(log_time) feature extractor ..."
./build/src/colmap/exe/colmap feature_extractor \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_mask ${PROJECT}/camera_mask.png \
  --ImageReader.camera_model OPENCV \
  --ImageReader.camera_params "550.0, 550.0, 480, 270, 0.12, -0.10, 0.00, 0.00" \
  --SiftExtraction.use_gpu 1 \
  --SiftExtraction.max_image_size 1024 \
  --SiftExtraction.max_num_features 1000 \
  --database_path ${PROJECT}/database.db \
  --image_path ${PROJECT}/images
echo "$(log_time) feature_extractor done."

echo "$(log_time) feature matcher ..."
./build/src/colmap/exe/colmap exhaustive_matcher \
  --SiftMatching.use_gpu 1 \
  --database_path ${PROJECT}/database.db
echo "$(log_time) feature exhaustive_matcher done."