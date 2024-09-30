#!/bin/sh

log_time() {
    date "+%Y-%m-%d %H:%M:%S:%3N"
}

PROJECT="${PWD}/proj"

echo "$(log_time) feature_extractor..."
./build/src/exe/colmap feature_extractor \
  --SiftExtraction.use_gpu 0 \
  --SiftExtraction.max_image_size 1024 \
  --SiftExtraction.max_num_features 2000 \
  --database_path ${PROJECT}/database.db \
  --image_path ${PROJECT}/images
echo "$(log_time) feature_extractor done."

echo "$(log_time) feature exhaustive_matcher..."
./build/src/exe/colmap exhaustive_matcher \
  --SiftMatching.use_gpu 0 \
  --database_path ${PROJECT}/database.db
echo "$(log_time) feature exhaustive_matcher done."

mkdir -p ${PROJECT}/sparse
echo "$(log_time) colmap mapper..."
./build/src/exe/colmap mapper \
  --Mapper.ba_local_max_num_iterations 25 \
  --Mapper.ba_global_max_num_iterations 50 \
  --database_path ${PROJECT}/database.db \
  --image_path ${PROJECT}/images \
  --output_path ${PROJECT}/sparse
echo "$(log_time) colmap mapper done."

# echo "$(log_time) glomap mapper..."
# glomap mapper \
#   --database_path ${PROJECT}/database.db \
#   --image_path ${PROJECT}/images \
#   --output_path ${PROJECT}/sparse
# echo "$(log_time) glomap mapper done."