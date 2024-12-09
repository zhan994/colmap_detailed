#!/bin/sh

# 固定内参共享的SFM，并GPS信息对齐
# Zhihao Zhan

log_time() {
    date "+%Y-%m-%d %H:%M:%S:%3N"
}

PROJECT="${PWD}/proj"
python3 work/python/camera_mask.py 960 540 200 250 ${PROJECT}/camera_mask.png
python3 work/python/exif_to_gps.py ${PROJECT}/images ${PROJECT}/gps.txt

echo "$(log_time) feature extractor ..."
./build/src/colmap/exe/colmap feature_extractor \
  --ImageReader.single_camera 1 \
  --ImageReader.camera_mask ${PROJECT}/camera_mask.png \
  --ImageReader.camera_model OPENCV \
  --ImageReader.camera_params "3716.2575775281393, 3716.1114704670836, 2639.5648205725929, 1962.1973365330105, -0.10287103597145282, -0.017408108357317358, -5.9160707243943791e-05, -0.00010785254872742507" \
  --SiftExtraction.use_gpu 1 \
  --SiftExtraction.max_image_size 1024 \
  --SiftExtraction.max_num_features 3000 \
  --database_path ${PROJECT}/database.db \
  --image_path ${PROJECT}/images
echo "$(log_time) feature_extractor done."

echo "$(log_time) feature matcher ..."
./build/src/colmap/exe/colmap exhaustive_matcher\
  --SiftMatching.use_gpu 1 \
  --database_path ${PROJECT}/database.db
echo "$(log_time) feature exhaustive_matcher done."

mkdir -p ${PROJECT}/sparse

echo "$(log_time) colmap mapper ..."
./build/src/colmap/exe/colmap mapper \
  --Mapper.ba_refine_principal_point 0 \
  --Mapper.ba_refine_focal_length 0 \
  --Mapper.ba_refine_extra_params 0 \
  --Mapper.ba_local_max_num_iterations 25 \
  --Mapper.ba_global_max_num_iterations 50 \
  --database_path ${PROJECT}/database.db \
  --image_path ${PROJECT}/images \
  --output_path ${PROJECT}/sparse
echo "$(log_time) colmap mapper done."

# echo "$(log_time) glomap mapper ..."
# ../simple_glomap/build/simple_glomap mapper \
#   --database_path ${PROJECT}/database.db \
#   --image_path ${PROJECT}/images \
#   --output_path ${PROJECT}/sparse
# echo "$(log_time) glomap mapper done."

mkdir -p ${PROJECT}/sparse/0_aligned_enu
echo "$(log_time) ENU align ..."
./build/src/colmap/exe/colmap model_aligner \
    --input_path  ${PROJECT}/sparse/0 \
    --output_path  ${PROJECT}/sparse/0_aligned_enu \
    --ref_images_path ${PROJECT}/gps.txt \
    --ref_is_gps 1 \
    --alignment_type enu \
    --alignment_max_error 3
echo "$(log_time) ENU align done."

echo "$(log_time) export ENU as txt ..."
./build/src/colmap/exe/colmap model_converter \
    --input_path ${PROJECT}/sparse/0_aligned_enu \
    --output_path ${PROJECT}/sparse/0_aligned_enu \
    --output_type TXT
echo "$(log_time) export ENU as txt done."

mkdir -p ${PROJECT}/sparse/0_aligned_ecef
echo "$(log_time) ECEF align ..."
./build/src/colmap/exe/colmap model_aligner \
    --input_path  ${PROJECT}/sparse/0 \
    --output_path  ${PROJECT}/sparse/0_aligned_ecef \
    --ref_images_path ${PROJECT}/gps.txt \
    --ref_is_gps 1 \
    --alignment_type ecef \
    --alignment_max_error 3
echo "$(log_time) ECEF align done."

echo "$(log_time) export ECEF as txt ..."
./build/src/colmap/exe/colmap model_converter \
    --input_path ${PROJECT}/sparse/0_aligned_ecef \
    --output_path ${PROJECT}/sparse/0_aligned_ecef \
    --output_type TXT
echo "$(log_time) export ECEF as txt done."

echo "$(log_time) convert camera pose from Tcw to Twc ..."
python3 work/python/colmap_pose.py \
  ${PROJECT}/sparse/0_aligned_enu/images.txt \
  ${PROJECT}/sparse/0_aligned_enu/images_Twc.txt
python3 work/python/colmap_pose.py \
  ${PROJECT}/sparse/0_aligned_ecef/images.txt \
  ${PROJECT}/sparse/0_aligned_ecef/images_Twc.txt
echo "$(log_time) camera pose conversion done."

echo "$(log_time) update images_Twc.txt ..."
python3 work/python/update_Twc.py \
  ${PROJECT}/gps.txt \
  ${PROJECT}/sparse/0_aligned_ecef/images_Twc.txt \
  ${PROJECT}/sparse/0_aligned_ecef/points3D.txt \
  ${PROJECT}/sparse/0_aligned_ecef/images_Twc_updated.txt
echo "$(log_time) update images_Twc.txt done."

echo "$(log_time) generate photo_record_quat.csv ..."
python3 work/python/colmap_quat_csv.py \
  ${PROJECT}/sparse/0_aligned_ecef/images_Twc_updated.txt \
  ${PROJECT}/sparse/0_aligned_enu/images_Twc.txt \
  ${PROJECT}/photo_record_quat.csv
echo "$(log_time) photo_record_quat.csv generation done."
