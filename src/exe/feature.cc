// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "exe/feature.h"

#include "base/camera_models.h"
#include "base/image_reader.h"
#include "exe/gui.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "util/misc.h"
#include "util/opengl_utils.h"
#include "util/option_manager.h"

namespace colmap {
namespace {

/**
 * \brief // api: 检查相机模型参数设置是否合理
 *
 * \param camera_model 相机模型名称
 * \param params 参数
 * \return true
 * \return false
 */
bool VerifyCameraParams(const std::string& camera_model,
                        const std::string& params) {
  // step: 1 检查模型是否存在
  if (!ExistsCameraModelWithName(camera_model)) {
    std::cerr << "ERROR: Camera model does not exist" << std::endl;
    return false;
  }

  // step: 2 转为模型id
  const std::vector<double> camera_params = CSVToVector<double>(params);
  const int camera_model_id = CameraModelNameToId(camera_model);

  // step: 3 根据模型id来判断当前模型参数设置是否合理
  if (camera_params.size() > 0 &&
      !CameraModelVerifyParams(camera_model_id, camera_params)) {
    std::cerr << "ERROR: Invalid camera parameters" << std::endl;
    return false;
  }
  return true;
}

/**
 * \brief // api: 检查sift设置是否可行
 *
 * \param use_gpu 是否使用
 * \return true
 * \return false
 */
bool VerifySiftGPUParams(const bool use_gpu) {
#if !defined(CUDA_ENABLED) && !defined(OPENGL_ENABLED)
  // note: 若不支持gpu运算，use_gpu=true则会报错
  if (use_gpu) {
    std::cerr << "ERROR: Cannot use Sift GPU without CUDA or OpenGL support; "
                 "set SiftExtraction.use_gpu or SiftMatching.use_gpu to false."
              << std::endl;
    return false;
  }
#endif
  return true;
}

}  // namespace

void UpdateImageReaderOptionsFromCameraMode(ImageReaderOptions& options,
                                            CameraMode mode) {
  switch (mode) {
    case CameraMode::AUTO:
      options.single_camera = false;
      options.single_camera_per_folder = false;
      options.single_camera_per_image = false;
      break;
    case CameraMode::SINGLE:
      options.single_camera = true;
      options.single_camera_per_folder = false;
      options.single_camera_per_image = false;
      break;
    case CameraMode::PER_FOLDER:
      options.single_camera = false;
      options.single_camera_per_folder = true;
      options.single_camera_per_image = false;
      break;
    case CameraMode::PER_IMAGE:
      options.single_camera = false;
      options.single_camera_per_folder = false;
      options.single_camera_per_image = true;
      break;
  }
}

// api: 特征检测
int RunFeatureExtractor(int argc, char** argv) {
  // step: 1 参数解析
  std::string image_list_path;                       // 默认 ""
  int camera_mode = -1;                              // 默认 -1
  std::string descriptor_normalization = "l1_root";  // 默认 l1_root

  OptionManager options;
  options.AddDatabaseOptions();  // 新增数据库参数
  options.AddImageOptions();     // 新增图片参数
  options.AddDefaultOption("camera_mode", &camera_mode);  // 相机模式
  options.AddDefaultOption("image_list_path", &image_list_path);
  // 描述子计算范数
  options.AddDefaultOption("descriptor_normalization",
                           &descriptor_normalization, "{'l1_root', 'l2'}");
  // 特征提取参数，包含image_reader和sift_extraction
  options.AddExtractionOptions();
  options.Parse(argc, argv);  // 解析命令行参数

  // step: 2 图片读取参数
  ImageReaderOptions reader_options = *options.image_reader;
  reader_options.database_path = *options.database_path;
  reader_options.image_path = *options.image_path;
  // 更新image_reader参数
  if (camera_mode >= 0) {
    UpdateImageReaderOptionsFromCameraMode(reader_options,
                                           (CameraMode)camera_mode);
  }

  // step: 3 描述子计算范数
  StringToLower(&descriptor_normalization);
  if (descriptor_normalization == "l1_root") {
    options.sift_extraction->normalization =
        SiftExtractionOptions::Normalization::L1_ROOT;
  } else if (descriptor_normalization == "l2") {
    options.sift_extraction->normalization =
        SiftExtractionOptions::Normalization::L2;
  } else {
    std::cerr << "ERROR: Invalid `descriptor_normalization`" << std::endl;
    return EXIT_FAILURE;
  }

  // step: 4 图片列表更新
  if (!image_list_path.empty()) {
    reader_options.image_list = ReadTextFileLines(image_list_path);
    if (reader_options.image_list.empty()) {
      return EXIT_SUCCESS;
    }
  }

  // step: 5 通过相机模型名称检查是否存在当前相机模型
  if (!ExistsCameraModelWithName(reader_options.camera_model)) {
    std::cerr << "ERROR: Camera model does not exist" << std::endl;
  }

  // step: 6 检查相机模型的参数是否合理
  if (!VerifyCameraParams(reader_options.camera_model,
                          reader_options.camera_params)) {
    return EXIT_FAILURE;
  }

  // step: 7 检查sift的gpu设置
  if (!VerifySiftGPUParams(options.sift_extraction->use_gpu)) {
    return EXIT_FAILURE;
  }

  // step: 8 是否同时gpu和opengl
  std::unique_ptr<QApplication> app;
  if (options.sift_extraction->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  // step: 9 开始sift
  SiftFeatureExtractor feature_extractor(reader_options,
                                         *options.sift_extraction);
  if (options.sift_extraction->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(&feature_extractor);
  } else {
    std::cout << " >>>>>>>>>>>> Start SIFT,,," << std::endl;
    feature_extractor.Start();
    feature_extractor.Wait();
  }

  return EXIT_SUCCESS;
}

int RunFeatureImporter(int argc, char** argv) {
  std::string import_path;
  std::string image_list_path;
  int camera_mode = -1;

  OptionManager options;
  options.AddDatabaseOptions();
  options.AddImageOptions();
  options.AddDefaultOption("camera_mode", &camera_mode);
  options.AddRequiredOption("import_path", &import_path);
  options.AddDefaultOption("image_list_path", &image_list_path);
  options.AddExtractionOptions();
  options.Parse(argc, argv);

  ImageReaderOptions reader_options = *options.image_reader;
  reader_options.database_path = *options.database_path;
  reader_options.image_path = *options.image_path;

  if (camera_mode >= 0) {
    UpdateImageReaderOptionsFromCameraMode(reader_options,
                                           (CameraMode)camera_mode);
  }

  if (!image_list_path.empty()) {
    reader_options.image_list = ReadTextFileLines(image_list_path);
    if (reader_options.image_list.empty()) {
      return EXIT_SUCCESS;
    }
  }

  if (!VerifyCameraParams(reader_options.camera_model,
                          reader_options.camera_params)) {
    return EXIT_FAILURE;
  }

  FeatureImporter feature_importer(reader_options, import_path);
  feature_importer.Start();
  feature_importer.Wait();

  return EXIT_SUCCESS;
}

int RunExhaustiveMatcher(int argc, char** argv) {
  OptionManager options;
  options.AddDatabaseOptions();
  options.AddExhaustiveMatchingOptions();
  options.Parse(argc, argv);

  if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<QApplication> app;
  if (options.sift_matching->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  ExhaustiveFeatureMatcher feature_matcher(*options.exhaustive_matching,
                                           *options.sift_matching,
                                           *options.database_path);

  if (options.sift_matching->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(&feature_matcher);
  } else {
    feature_matcher.Start();
    feature_matcher.Wait();
  }

  return EXIT_SUCCESS;
}

int RunMatchesImporter(int argc, char** argv) {
  std::string match_list_path;
  std::string match_type = "pairs";

  OptionManager options;
  options.AddDatabaseOptions();
  options.AddRequiredOption("match_list_path", &match_list_path);
  options.AddDefaultOption("match_type", &match_type,
                           "{'pairs', 'raw', 'inliers'}");
  options.AddMatchingOptions();
  options.Parse(argc, argv);

  if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<QApplication> app;
  if (options.sift_matching->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  std::unique_ptr<Thread> feature_matcher;
  if (match_type == "pairs") {
    ImagePairsMatchingOptions matcher_options;
    matcher_options.match_list_path = match_list_path;
    feature_matcher.reset(new ImagePairsFeatureMatcher(
        matcher_options, *options.sift_matching, *options.database_path));
  } else if (match_type == "raw" || match_type == "inliers") {
    FeaturePairsMatchingOptions matcher_options;
    matcher_options.match_list_path = match_list_path;
    matcher_options.verify_matches = match_type == "raw";
    feature_matcher.reset(new FeaturePairsFeatureMatcher(
        matcher_options, *options.sift_matching, *options.database_path));
  } else {
    std::cerr << "ERROR: Invalid `match_type`";
    return EXIT_FAILURE;
  }

  if (options.sift_matching->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(feature_matcher.get());
  } else {
    feature_matcher->Start();
    feature_matcher->Wait();
  }

  return EXIT_SUCCESS;
}

int RunSequentialMatcher(int argc, char** argv) {
  OptionManager options;
  options.AddDatabaseOptions();
  options.AddSequentialMatchingOptions();
  options.Parse(argc, argv);

  if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<QApplication> app;
  if (options.sift_matching->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  SequentialFeatureMatcher feature_matcher(*options.sequential_matching,
                                           *options.sift_matching,
                                           *options.database_path);

  if (options.sift_matching->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(&feature_matcher);
  } else {
    feature_matcher.Start();
    feature_matcher.Wait();
  }

  return EXIT_SUCCESS;
}

int RunSpatialMatcher(int argc, char** argv) {
  OptionManager options;
  options.AddDatabaseOptions();
  options.AddSpatialMatchingOptions();
  options.Parse(argc, argv);

  if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<QApplication> app;
  if (options.sift_matching->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  SpatialFeatureMatcher feature_matcher(*options.spatial_matching,
                                        *options.sift_matching,
                                        *options.database_path);

  if (options.sift_matching->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(&feature_matcher);
  } else {
    feature_matcher.Start();
    feature_matcher.Wait();
  }

  return EXIT_SUCCESS;
}

int RunTransitiveMatcher(int argc, char** argv) {
  OptionManager options;
  options.AddDatabaseOptions();
  options.AddTransitiveMatchingOptions();
  options.Parse(argc, argv);

  if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<QApplication> app;
  if (options.sift_matching->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  TransitiveFeatureMatcher feature_matcher(*options.transitive_matching,
                                           *options.sift_matching,
                                           *options.database_path);

  if (options.sift_matching->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(&feature_matcher);
  } else {
    feature_matcher.Start();
    feature_matcher.Wait();
  }

  return EXIT_SUCCESS;
}

int RunVocabTreeMatcher(int argc, char** argv) {
  OptionManager options;
  options.AddDatabaseOptions();
  options.AddVocabTreeMatchingOptions();
  options.Parse(argc, argv);

  if (!VerifySiftGPUParams(options.sift_matching->use_gpu)) {
    return EXIT_FAILURE;
  }

  std::unique_ptr<QApplication> app;
  if (options.sift_matching->use_gpu && kUseOpenGL) {
    app.reset(new QApplication(argc, argv));
  }

  VocabTreeFeatureMatcher feature_matcher(*options.vocab_tree_matching,
                                          *options.sift_matching,
                                          *options.database_path);

  if (options.sift_matching->use_gpu && kUseOpenGL) {
    RunThreadWithOpenGLContext(&feature_matcher);
  } else {
    feature_matcher.Start();
    feature_matcher.Wait();
  }

  return EXIT_SUCCESS;
}

}  // namespace colmap
