#!/bin/bash
DATASET_PATH=xxxxxxxxxxxxxxxxxxxxxxx

colmap feature_extractor \
  --database_path $DATASET_PATH/database.db \
  --image_path $DATASET_PATH/images \
  --SiftExtraction.max_num_features 16384 \
  --SiftExtraction.estimate_affine_shape=true \
  --SiftExtraction.domain_size_pooling=true \
  --SiftExtraction.use_gpu 1 \
  --SiftExtraction.gpu_index 0

colmap exhaustive_matcher \ 
  --database_path $DATASET_PATH/database.db \
  --SiftMatching.guided_matching true

mkdir $DATASET_PATH/sparse

colmap mapper \
  --database_path $DATASET_PATH/database.db \
  --image_path $DATASET_PATH/images \
  --output_path $DATASET_PATH/sparse \
  --Mapper.min_num_matches 30 \
  --Mapper.abs_pose_min_num_inliers 40 \
  --Mapper.init_max_forward_motion 0.95 \
  --Mapper.filter_min_tri_angle 2 \
  --Mapper.tri_create_max_angle_error 1 \
  --Mapper.tri_continue_max_angle_error 1 \
  --Mapper.multiple_models 0 