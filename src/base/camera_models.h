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

#ifndef COLMAP_SRC_BASE_CAMERA_MODELS_H_
#define COLMAP_SRC_BASE_CAMERA_MODELS_H_

#include <cfloat>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>

namespace colmap {

// This file defines several different camera models and arbitrary new camera
// models can be added by the following steps:
//
//  1. Add a new struct in this file which implements all the necessary methods.
//  2. Define an unique model_name and model_id for the camera model.
//  3. Add camera model to `CAMERA_MODEL_CASES` macro in this file.
//  4. Add new template specialization of test case for camera model to
//     `camera_models_test.cc`.
//
// A camera model can have three different types of camera parameters: focal
// length, principal point, extra parameters (distortion parameters). The
// parameter array is split into different groups, so that we can enable or
// disable the refinement of the individual groups during bundle adjustment. It
// is up to the camera model to access the parameters correctly (it is free to
// do so in an arbitrary manner) - the parameters are not accessed from outside.
//
// A camera model must have the following methods:
//
//  - `WorldToImage`: transform normalized camera coordinates to image
//    coordinates (the inverse of `ImageToWorld`). Assumes that the world
//    coordinates are given as (u, v, 1).
//  - `ImageToWorld`: transform image coordinates to normalized camera
//    coordinates (the inverse of `WorldToImage`). Produces world coordinates
//    as (u, v, 1).
//  - `ImageToWorldThreshold`: transform a threshold given in pixels to
//    normalized units (e.g. useful for reprojection error thresholds).
//
// Whenever you specify the camera parameters in a list, they must appear
// exactly in the order as they are accessed in the defined model struct.
//
// The camera models follow the convention that the upper left image corner has
// the coordinate (0, 0), the lower right corner (width, height), i.e. that
// the upper left pixel center has coordinate (0.5, 0.5) and the lower right
// pixel center has the coordinate (width - 0.5, height - 0.5).

static const int kInvalidCameraModelId = -1;

// note: 通过宏定义来声明一种相机模型的所有类的静态成员声明
// api: InitializeFocalLengthIdxs表示初始化参数中焦距的参数索引
// api: InitializePrincipalPointIdxs表示初始化参数中中心的参数索引
// api: InitializeExtraParamsIdxs表示初始化参数中其他参数的索引
// api: InitializeParams表示初始化f,cx,cy
// api: WorldToImage将归一化的相机坐标系转换到像素坐标系，假设世界相机系为(u, v,
// 1) api: ImageToWorld将像素坐标系转换到归一化的相机坐标系，生成世界相机系为(u,
// v, 1) api: Distortion根据畸变参数进行像素去畸变
#ifndef CAMERA_MODEL_DEFINITIONS
#define CAMERA_MODEL_DEFINITIONS(model_id_value, model_name_value,             \
                                 num_params_value)                             \
  static const int kModelId = model_id_value;                                  \
  static const size_t kNumParams = num_params_value;                           \
  static const int model_id;                                                   \
  static const std::string model_name;                                         \
  static const size_t num_params;                                              \
  static const std::string params_info;                                        \
  static const std::vector<size_t> focal_length_idxs;                          \
  static const std::vector<size_t> principal_point_idxs;                       \
  static const std::vector<size_t> extra_params_idxs;                          \
                                                                               \
  static inline int InitializeModelId() { return model_id_value; };            \
  static inline std::string InitializeModelName() {                            \
    return model_name_value;                                                   \
  };                                                                           \
  static inline size_t InitializeNumParams() { return num_params_value; };     \
  static inline std::string InitializeParamsInfo();                            \
  static inline std::vector<size_t> InitializeFocalLengthIdxs();               \
  static inline std::vector<size_t> InitializePrincipalPointIdxs();            \
  static inline std::vector<size_t> InitializeExtraParamsIdxs();               \
  static inline std::vector<double> InitializeParams(                          \
      const double focal_length, const size_t width, const size_t height);     \
                                                                               \
  template <typename T>                                                        \
  static void WorldToImage(const T* params, const T u, const T v, T* x, T* y); \
  template <typename T>                                                        \
  static void ImageToWorld(const T* params, const T x, const T y, T* u, T* v); \
  template <typename T>                                                        \
  static void Distortion(const T* extra_params, const T u, const T v, T* du,   \
                         T* dv);
#endif

// note: 宏定义所有相机模型
#ifndef CAMERA_MODEL_CASES
#define CAMERA_MODEL_CASES                          \
  CAMERA_MODEL_CASE(SimplePinholeCameraModel)       \
  CAMERA_MODEL_CASE(PinholeCameraModel)             \
  CAMERA_MODEL_CASE(SimpleRadialCameraModel)        \
  CAMERA_MODEL_CASE(SimpleRadialFisheyeCameraModel) \
  CAMERA_MODEL_CASE(RadialCameraModel)              \
  CAMERA_MODEL_CASE(RadialFisheyeCameraModel)       \
  CAMERA_MODEL_CASE(OpenCVCameraModel)              \
  CAMERA_MODEL_CASE(OpenCVFisheyeCameraModel)       \
  CAMERA_MODEL_CASE(FullOpenCVCameraModel)          \
  CAMERA_MODEL_CASE(FOVCameraModel)                 \
  CAMERA_MODEL_CASE(ThinPrismFisheyeCameraModel)
#endif

// note: 将上述所有相机模型表示为switch的使用方式
#ifndef CAMERA_MODEL_SWITCH_CASES
#define CAMERA_MODEL_SWITCH_CASES         \
  CAMERA_MODEL_CASES                      \
  default:                                \
    CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION \
    break;
#endif

#define CAMERA_MODEL_DOES_NOT_EXIST_EXCEPTION \
  throw std::domain_error("Camera model does not exist");

// The "Curiously Recurring Template Pattern" (CRTP) is used here, so that we
// can reuse some shared functionality between all camera models -
// defined in the BaseCameraModel.
// note: 相机模型的基础类
template <typename CameraModel>
struct BaseCameraModel {
  /**
   * \brief // api: 是否有假参数
   *
   * \tparam T
   * \param model_id
   * \param params 参数
   * \param width 成像宽
   * \param height 成像高
   * \param min_focal_length_ratio 最小焦距比例
   * \param max_focal_length_ratio 最大焦距比例
   * \param max_extra_param 最大参数值m
   * \return true
   * \return false
   */
  template <typename T>
  static inline bool HasBogusParams(const std::vector<T>& params,
                                    const size_t width, const size_t height,
                                    const T min_focal_length_ratio,
                                    const T max_focal_length_ratio,
                                    const T max_extra_param);

  /**
   * \brief // api: 是否有假焦距
   *
   * \tparam T
   * \param params 参数
   * \param width 成像宽
   * \param height 成像高
   * \param min_focal_length_ratio 最小焦距比例
   * \param max_focal_length_ratio 最大焦距比例
   * \return true
   * \return false
   */
  template <typename T>
  static inline bool HasBogusFocalLength(const std::vector<T>& params,
                                         const size_t width,
                                         const size_t height,
                                         const T min_focal_length_ratio,
                                         const T max_focal_length_ratio);

  /**
   * \brief // api: 是否有假像中心
   *
   * \tparam T
   * \param params 参数
   * \param width 成像宽
   * \param height 成像高
   * \return true
   * \return false
   */
  template <typename T>
  static inline bool HasBogusPrincipalPoint(const std::vector<T>& params,
                                            const size_t width,
                                            const size_t height);

  /**
   * \brief // api: 是否有假的额外参数
   *
   * \tparam T
   * \param params 参数
   * \param max_extra_param 最大参数值
   * \return true
   * \return false
   */
  template <typename T>
  static inline bool HasBogusExtraParams(const std::vector<T>& params,
                                         const T max_extra_param);

  /**
   * \brief // api: 像素转世界相机系的阈值
   *
   * \tparam T
   * \param params 参数
   * \param threshold 像素阈值
   * \return T
   */
  template <typename T>
  static inline T ImageToWorldThreshold(const T* params, const T threshold);

  /**
   * \brief // api: 迭代去畸变
   *
   * \tparam T
   * \param params 参数
   * \param u
   * \param v
   */
  template <typename T>
  static inline void IterativeUndistortion(const T* params, T* u, T* v);
};

// Simple Pinhole camera model.
//
// No Distortion is assumed. Only focal length and principal point is modeled.
//
// Parameter list is expected in the following order:
//
//   f, cx, cy
//
// See https://en.wikipedia.org/wiki/Pinhole_camera_model
// note: 简易pinhole，id=0，3个参数f, cx, cy
struct SimplePinholeCameraModel
    : public BaseCameraModel<SimplePinholeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(0, "SIMPLE_PINHOLE", 3)
};

// Pinhole camera model.
//
// No Distortion is assumed. Only focal length and principal point is modeled.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy
//
// See https://en.wikipedia.org/wiki/Pinhole_camera_model
// note: pinhole，id=1, 4个参数fx, fy, cx, cy
struct PinholeCameraModel : public BaseCameraModel<PinholeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(1, "PINHOLE", 4)
};

// Simple camera model with one focal length and one radial distortion
// parameter.
//
// This model is similar to the camera model that VisualSfM uses with the
// difference that the distortion here is applied to the projections and
// not to the measurements.
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k
//
// note: 简易径向，id=2，4个参数f, cx, cy, k
struct SimpleRadialCameraModel
    : public BaseCameraModel<SimpleRadialCameraModel> {
  CAMERA_MODEL_DEFINITIONS(2, "SIMPLE_RADIAL", 4)
};

// Simple camera model with one focal length and two radial distortion
// parameters.
//
// This model is equivalent to the camera model that Bundler uses
// (except for an inverse z-axis in the camera coordinate system).
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k1, k2
//
// note: 径向，id=3，5个参数f, cx, cy, k1, k2
struct RadialCameraModel : public BaseCameraModel<RadialCameraModel> {
  CAMERA_MODEL_DEFINITIONS(3, "RADIAL", 5)
};

// OpenCV camera model.
//
// Based on the pinhole camera model. Additionally models radial and
// tangential distortion (up to 2nd degree of coefficients). Not suitable for
// large radial distortions of fish-eye cameras.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, p1, p2
//
// See
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
// note: opencv，id=4，8个参数fx, fy, cx, cy, k1, k2, p1, p2
struct OpenCVCameraModel : public BaseCameraModel<OpenCVCameraModel> {
  CAMERA_MODEL_DEFINITIONS(4, "OPENCV", 8)
};

// OpenCV fish-eye camera model.
//
// Based on the pinhole camera model. Additionally models radial and
// tangential Distortion (up to 2nd degree of coefficients). Suitable for
// large radial distortions of fish-eye cameras.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, k3, k4
//
// See
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
// note: opencv鱼眼，id=5，8个参数fx, fy, cx, cy, k1, k2, k3, k4
struct OpenCVFisheyeCameraModel
    : public BaseCameraModel<OpenCVFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(5, "OPENCV_FISHEYE", 8)
};

// Full OpenCV camera model.
//
// Based on the pinhole camera model. Additionally models radial and
// tangential Distortion.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
//
// See
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
// note: 完整opencv，id=6，12个参数fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5,
// k6
struct FullOpenCVCameraModel : public BaseCameraModel<FullOpenCVCameraModel> {
  CAMERA_MODEL_DEFINITIONS(6, "FULL_OPENCV", 12)
};

// FOV camera model.
//
// Based on the pinhole camera model. Additionally models radial distortion.
// This model is for example used by Project Tango for its equidistant
// calibration type.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, omega
//
// See:
// Frederic Devernay, Olivier Faugeras. Straight lines have to be straight:
// Automatic calibration and removal of distortion from scenes of structured
// environments. Machine vision and applications, 2001.
struct FOVCameraModel : public BaseCameraModel<FOVCameraModel> {
  CAMERA_MODEL_DEFINITIONS(7, "FOV", 5)

  template <typename T>
  static void Undistortion(const T* extra_params, const T u, const T v, T* du,
                           T* dv);
};

// Simple camera model with one focal length and one radial distortion
// parameter, suitable for fish-eye cameras.
//
// This model is equivalent to the OpenCVFisheyeCameraModel but has only one
// radial distortion coefficient.
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k
//
struct SimpleRadialFisheyeCameraModel
    : public BaseCameraModel<SimpleRadialFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(8, "SIMPLE_RADIAL_FISHEYE", 4)
};

// Simple camera model with one focal length and two radial distortion
// parameters, suitable for fish-eye cameras.
//
// This model is equivalent to the OpenCVFisheyeCameraModel but has only two
// radial distortion coefficients.
//
// Parameter list is expected in the following order:
//
//    f, cx, cy, k1, k2
//
struct RadialFisheyeCameraModel
    : public BaseCameraModel<RadialFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(9, "RADIAL_FISHEYE", 5)
};

// Camera model with radial and tangential distortion coefficients and
// additional coefficients accounting for thin-prism distortion.
//
// This camera model is described in
//
//    "Camera Calibration with Distortion Models and Accuracy Evaluation",
//    J Weng et al., TPAMI, 1992.
//
// Parameter list is expected in the following order:
//
//    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
//
struct ThinPrismFisheyeCameraModel
    : public BaseCameraModel<ThinPrismFisheyeCameraModel> {
  CAMERA_MODEL_DEFINITIONS(10, "THIN_PRISM_FISHEYE", 12)
};

// api: 通过名称来检查相机模型是否存在
bool ExistsCameraModelWithName(const std::string& model_name);
// api: 通过id来检查相机模型是否存在
bool ExistsCameraModelWithId(const int model_id);

// api: 将相机模型名转为id
int CameraModelNameToId(const std::string& model_name);
// api: 将相机id转为模型名
std::string CameraModelIdToName(const int model_id);

/**
 * \brief // api: 初始化相机模型参数
 *
 * \param model_id
 * \param focal_length 焦距
 * \param width 图片宽
 * \param height 图片高
 * \return std::vector<double> 所有相机模型的参数
 */
std::vector<double> CameraModelInitializeParams(const int model_id,
                                                const double focal_length,
                                                const size_t width,
                                                const size_t height);

/**
 * \brief // api: 相机模型参数的信息
 *
 * \param model_id
 * \return std::string 参数解释
 */
std::string CameraModelParamsInfo(const int model_id);

/**
 * \brief // api: 相机模型中的相关参数的索引
 *
 * \param model_id
 * \return const std::vector<size_t>& 索引值列表
 */
const std::vector<size_t>& CameraModelFocalLengthIdxs(const int model_id);
const std::vector<size_t>& CameraModelPrincipalPointIdxs(const int model_id);
const std::vector<size_t>& CameraModelExtraParamsIdxs(const int model_id);

// api: 相机模型的参数个数
size_t CameraModelNumParams(const int model_id);

/**
 * \brief // api: 验证模型参数是否有效，参数数量是否和设置一致
 *
 * \param model_id
 * \param params 参数
 * \return true 有效
 * \return false 无效
 */
bool CameraModelVerifyParams(const int model_id,
                             const std::vector<double>& params);

/**
 * \brief // api: 验证模型是否有假的参数
 *
 * \param model_id
 * \param params 参数
 * \param width 成像宽
 * \param height 成像高
 * \param min_focal_length_ratio 最小焦距比例
 * \param max_focal_length_ratio 最大焦距比例
 * \param max_extra_param 最大参数值
 * \return true
 * \return false
 */
bool CameraModelHasBogusParams(const int model_id,
                               const std::vector<double>& params,
                               const size_t width, const size_t height,
                               const double min_focal_length_ratio,
                               const double max_focal_length_ratio,
                               const double max_extra_param);

/**
 * \brief // api: 通过指定相机模型id来转换世界相机系到像素坐标系
 *
 * \param model_id
 * \param params 参数
 * \param u 世界相机系 (u, v, 1).
 * \param v
 * \param x 像素系 (x, y, 1)
 * \param y
 */
inline void CameraModelWorldToImage(const int model_id,
                                    const std::vector<double>& params,
                                    const double u, const double v, double* x,
                                    double* y);

/**
 * \brief // api: 通过指定相机模型id来转换像素坐标系到世界相机系
 *
 * \param model_id
 * \param params 参数
 * \param x 像素系 (x, y, 1)
 * \param y
 * \param u 世界相机系 (u, v, 1).
 * \param v
 */
inline void CameraModelImageToWorld(const int model_id,
                                    const std::vector<double>& params,
                                    const double x, const double y, double* u,
                                    double* v);

/**
 * \brief // api: 通过指定相机模型id来计算像素转世界相机系的阈值
 *
 * \param model_id
 * \param params 参数
 * \param threshold 像素系阈值
 * \return double 世界相机系阈值
 */
inline double CameraModelImageToWorldThreshold(
    const int model_id, const std::vector<double>& params,
    const double threshold);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// note: BaseCameraModel

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusParams(
    const std::vector<T>& params, const size_t width, const size_t height,
    const T min_focal_length_ratio, const T max_focal_length_ratio,
    const T max_extra_param) {
  if (HasBogusPrincipalPoint(params, width, height)) {
    return true;
  }

  if (HasBogusFocalLength(params, width, height, min_focal_length_ratio,
                          max_focal_length_ratio)) {
    return true;
  }

  if (HasBogusExtraParams(params, max_extra_param)) {
    return true;
  }

  return false;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusFocalLength(
    const std::vector<T>& params, const size_t width, const size_t height,
    const T min_focal_length_ratio, const T max_focal_length_ratio) {
  const size_t max_size = std::max(width, height);

  for (const auto& idx : CameraModel::focal_length_idxs) {
    const T focal_length_ratio = params[idx] / max_size;
    if (focal_length_ratio < min_focal_length_ratio ||
        focal_length_ratio > max_focal_length_ratio) {
      return true;
    }
  }

  return false;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusPrincipalPoint(
    const std::vector<T>& params, const size_t width, const size_t height) {
  const T cx = params[CameraModel::principal_point_idxs[0]];
  const T cy = params[CameraModel::principal_point_idxs[1]];
  return cx < 0 || cx > width || cy < 0 || cy > height;
}

template <typename CameraModel>
template <typename T>
bool BaseCameraModel<CameraModel>::HasBogusExtraParams(
    const std::vector<T>& params, const T max_extra_param) {
  for (const auto& idx : CameraModel::extra_params_idxs) {
    if (std::abs(params[idx]) > max_extra_param) {
      return true;
    }
  }

  return false;
}

template <typename CameraModel>
template <typename T>
T BaseCameraModel<CameraModel>::ImageToWorldThreshold(const T* params,
                                                      const T threshold) {
  T mean_focal_length = 0;
  for (const auto& idx : CameraModel::focal_length_idxs) {
    mean_focal_length += params[idx];
  }
  mean_focal_length /= CameraModel::focal_length_idxs.size();
  return threshold / mean_focal_length;
}

template <typename CameraModel>
template <typename T>
void BaseCameraModel<CameraModel>::IterativeUndistortion(const T* params, T* u,
                                                         T* v) {
  // Parameters for Newton iteration using numerical differentiation with
  // central differences, 100 iterations should be enough even for complex
  // camera models with higher order terms.
  const size_t kNumIterations = 100;
  const double kMaxStepNorm = 1e-10;
  const double kRelStepSize = 1e-6;

  Eigen::Matrix2d J;
  const Eigen::Vector2d x0(*u, *v);
  Eigen::Vector2d x(*u, *v);
  Eigen::Vector2d dx;
  Eigen::Vector2d dx_0b;
  Eigen::Vector2d dx_0f;
  Eigen::Vector2d dx_1b;
  Eigen::Vector2d dx_1f;

  for (size_t i = 0; i < kNumIterations; ++i) {
    const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(0)));
    const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                  std::abs(kRelStepSize * x(1)));
    CameraModel::Distortion(params, x(0), x(1), &dx(0), &dx(1));
    CameraModel::Distortion(params, x(0) - step0, x(1), &dx_0b(0), &dx_0b(1));
    CameraModel::Distortion(params, x(0) + step0, x(1), &dx_0f(0), &dx_0f(1));
    CameraModel::Distortion(params, x(0), x(1) - step1, &dx_1b(0), &dx_1b(1));
    CameraModel::Distortion(params, x(0), x(1) + step1, &dx_1f(0), &dx_1f(1));
    J(0, 0) = 1 + (dx_0f(0) - dx_0b(0)) / (2 * step0);
    J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2 * step1);
    J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2 * step0);
    J(1, 1) = 1 + (dx_1f(1) - dx_1b(1)) / (2 * step1);
    const Eigen::Vector2d step_x = J.inverse() * (x + dx - x0);
    x -= step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) {
      break;
    }
  }

  *u = x(0);
  *v = x(1);
}

////////////////////////////////////////////////////////////////////////////////
// note: SimplePinholeCameraModel

std::string SimplePinholeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy";
}

std::vector<size_t> SimplePinholeCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> SimplePinholeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> SimplePinholeCameraModel::InitializeExtraParamsIdxs() {
  return {};
}

std::vector<double> SimplePinholeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0};
}

template <typename T>
void SimplePinholeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // No Distortion

  // Transform to image coordinates
  *x = f * u + c1;
  *y = f * v + c2;
}

template <typename T>
void SimplePinholeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  *u = (x - c1) / f;
  *v = (y - c2) / f;
}

////////////////////////////////////////////////////////////////////////////////
// note: PinholeCameraModel

std::string PinholeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy";
}

std::vector<size_t> PinholeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> PinholeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> PinholeCameraModel::InitializeExtraParamsIdxs() {
  return {};
}

std::vector<double> PinholeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0};
}

template <typename T>
void PinholeCameraModel::WorldToImage(const T* params, const T u, const T v,
                                      T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // No Distortion

  // Transform to image coordinates
  *x = f1 * u + c1;
  *y = f2 * v + c2;
}

template <typename T>
void PinholeCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                      T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  *u = (x - c1) / f1;
  *v = (y - c2) / f2;
}

////////////////////////////////////////////////////////////////////////////////
// note: SimpleRadialCameraModel

std::string SimpleRadialCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k";
}

std::vector<size_t> SimpleRadialCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> SimpleRadialCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> SimpleRadialCameraModel::InitializeExtraParamsIdxs() {
  return {3};
}

std::vector<double> SimpleRadialCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0};
}

template <typename T>
void SimpleRadialCameraModel::WorldToImage(const T* params, const T u,
                                           const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void SimpleRadialCameraModel::ImageToWorld(const T* params, const T x,
                                           const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void SimpleRadialCameraModel::Distortion(const T* extra_params, const T u,
                                         const T v, T* du, T* dv) {
  const T k = extra_params[0];

  const T u2 = u * u;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k * r2;
  *du = u * radial;
  *dv = v * radial;
}

////////////////////////////////////////////////////////////////////////////////
// note: RadialCameraModel

std::string RadialCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k1, k2";
}

std::vector<size_t> RadialCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> RadialCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> RadialCameraModel::InitializeExtraParamsIdxs() {
  return {3, 4};
}

std::vector<double> RadialCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0, 0};
}

template <typename T>
void RadialCameraModel::WorldToImage(const T* params, const T u, const T v,
                                     T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void RadialCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                     T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void RadialCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                   T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];

  const T u2 = u * u;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k1 * r2 + k2 * r2 * r2;
  *du = u * radial;
  *dv = v * radial;
}

////////////////////////////////////////////////////////////////////////////////
// note: OpenCVCameraModel

std::string OpenCVCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, p1, p2";
}

std::vector<size_t> OpenCVCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> OpenCVCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> OpenCVCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7};
}

std::vector<double> OpenCVCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0, 0};
}

template <typename T>
void OpenCVCameraModel::WorldToImage(const T* params, const T u, const T v,
                                     T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  T du, dv;
  Distortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void OpenCVCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                     T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);
}

template <typename T>
void OpenCVCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                   T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k1 * r2 + k2 * r2 * r2;
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2);
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2);
}

////////////////////////////////////////////////////////////////////////////////
// note: OpenCVFisheyeCameraModel

std::string OpenCVFisheyeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, k3, k4";
}

std::vector<size_t> OpenCVFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> OpenCVFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> OpenCVFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7};
}

std::vector<double> OpenCVFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 0, 0, 0, 0};
}

template <typename T>
void OpenCVFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  T du, dv;
  Distortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void OpenCVFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);
}

template <typename T>
void OpenCVFisheyeCameraModel::Distortion(const T* extra_params, const T u,
                                          const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T k3 = extra_params[2];
  const T k4 = extra_params[3];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T theta6 = theta4 * theta2;
    const T theta8 = theta4 * theta4;
    const T thetad =
        theta * (T(1) + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// note: FullOpenCVCameraModel

std::string FullOpenCVCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6";
}

std::vector<size_t> FullOpenCVCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> FullOpenCVCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> FullOpenCVCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7, 8, 9, 10, 11};
}

std::vector<double> FullOpenCVCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length,
          focal_length,
          width / 2.0,
          height / 2.0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0};
}

template <typename T>
void FullOpenCVCameraModel::WorldToImage(const T* params, const T u, const T v,
                                         T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  T du, dv;
  Distortion(&params[4], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void FullOpenCVCameraModel::ImageToWorld(const T* params, const T x, const T y,
                                         T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);
}

template <typename T>
void FullOpenCVCameraModel::Distortion(const T* extra_params, const T u,
                                       const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];
  const T k3 = extra_params[4];
  const T k4 = extra_params[5];
  const T k5 = extra_params[6];
  const T k6 = extra_params[7];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T r4 = r2 * r2;
  const T r6 = r4 * r2;
  const T radial = (T(1) + k1 * r2 + k2 * r4 + k3 * r6) /
                   (T(1) + k4 * r2 + k5 * r4 + k6 * r6);
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2) - u;
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2) - v;
}

////////////////////////////////////////////////////////////////////////////////
// note: FOVCameraModel

std::string FOVCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, omega";
}

std::vector<size_t> FOVCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t> FOVCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> FOVCameraModel::InitializeExtraParamsIdxs() { return {4}; }

std::vector<double> FOVCameraModel::InitializeParams(const double focal_length,
                                                     const size_t width,
                                                     const size_t height) {
  return {focal_length, focal_length, width / 2.0, height / 2.0, 1e-2};
}

template <typename T>
void FOVCameraModel::WorldToImage(const T* params, const T u, const T v, T* x,
                                  T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Distortion
  Distortion(&params[4], u, v, x, y);

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void FOVCameraModel::ImageToWorld(const T* params, const T x, const T y, T* u,
                                  T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  const T uu = (x - c1) / f1;
  const T vv = (y - c2) / f2;

  // Undistortion
  Undistortion(&params[4], uu, vv, u, v);
}

template <typename T>
void FOVCameraModel::Distortion(const T* extra_params, const T u, const T v,
                                T* du, T* dv) {
  const T omega = extra_params[0];

  // Chosen arbitrarily.
  const T kEpsilon = T(1e-4);

  const T radius2 = u * u + v * v;
  const T omega2 = omega * omega;

  T factor;
  if (omega2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = atan(radius * 2 * tan(omega / 2)) / ...
    //                  (radius * omega);
    // simplify(taylor(factor, omega, 'order', 3))
    factor = (omega2 * radius2) / T(3) - omega2 / T(12) + T(1);
  } else if (radius2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = atan(radius * 2 * tan(omega / 2)) / ...
    //                  (radius * omega);
    // simplify(taylor(factor, radius, 'order', 3))
    const T tan_half_omega = ceres::tan(omega / T(2));
    factor = (T(-2) * tan_half_omega *
              (T(4) * radius2 * tan_half_omega * tan_half_omega - T(3))) /
             (T(3) * omega);
  } else {
    const T radius = ceres::sqrt(radius2);
    const T numerator = ceres::atan(radius * T(2) * ceres::tan(omega / T(2)));
    factor = numerator / (radius * omega);
  }

  *du = u * factor;
  *dv = v * factor;
}

template <typename T>
void FOVCameraModel::Undistortion(const T* extra_params, const T u, const T v,
                                  T* du, T* dv) {
  T omega = extra_params[0];

  // Chosen arbitrarily.
  const T kEpsilon = T(1e-4);

  const T radius2 = u * u + v * v;
  const T omega2 = omega * omega;

  T factor;
  if (omega2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = tan(radius * omega) / ...
    //                  (radius * 2*tan(omega/2));
    // simplify(taylor(factor, omega, 'order', 3))
    factor = (omega2 * radius2) / T(3) - omega2 / T(12) + T(1);
  } else if (radius2 < kEpsilon) {
    // Derivation of this case with Matlab:
    // syms radius omega;
    // factor(radius) = tan(radius * omega) / ...
    //                  (radius * 2*tan(omega/2));
    // simplify(taylor(factor, radius, 'order', 3))
    factor = (omega * (omega * omega * radius2 + T(3))) /
             (T(6) * ceres::tan(omega / T(2)));
  } else {
    const T radius = ceres::sqrt(radius2);
    const T numerator = ceres::tan(radius * omega);
    factor = numerator / (radius * T(2) * ceres::tan(omega / T(2)));
  }

  *du = u * factor;
  *dv = v * factor;
}

////////////////////////////////////////////////////////////////////////////////
// note: SimpleRadialFisheyeCameraModel

std::string SimpleRadialFisheyeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k";
}

std::vector<size_t>
SimpleRadialFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t>
SimpleRadialFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t>
SimpleRadialFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {3};
}

std::vector<double> SimpleRadialFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0};
}

template <typename T>
void SimpleRadialFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                                  const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void SimpleRadialFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                                  const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void SimpleRadialFisheyeCameraModel::Distortion(const T* extra_params,
                                                const T u, const T v, T* du,
                                                T* dv) {
  const T k = extra_params[0];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T thetad = theta * (T(1) + k * theta2);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// note: RadialFisheyeCameraModel

std::string RadialFisheyeCameraModel::InitializeParamsInfo() {
  return "f, cx, cy, k1, k2";
}

std::vector<size_t> RadialFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0};
}

std::vector<size_t> RadialFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {1, 2};
}

std::vector<size_t> RadialFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {3, 4};
}

std::vector<double> RadialFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length, width / 2.0, height / 2.0, 0, 0};
}

template <typename T>
void RadialFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                            const T v, T* x, T* y) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Distortion
  T du, dv;
  Distortion(&params[3], u, v, &du, &dv);
  *x = u + du;
  *y = v + dv;

  // Transform to image coordinates
  *x = f * *x + c1;
  *y = f * *y + c2;
}

template <typename T>
void RadialFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                            const T y, T* u, T* v) {
  const T f = params[0];
  const T c1 = params[1];
  const T c2 = params[2];

  // Lift points to normalized plane
  *u = (x - c1) / f;
  *v = (y - c2) / f;

  IterativeUndistortion(&params[3], u, v);
}

template <typename T>
void RadialFisheyeCameraModel::Distortion(const T* extra_params, const T u,
                                          const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];

  const T r = ceres::sqrt(u * u + v * v);

  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    const T theta2 = theta * theta;
    const T theta4 = theta2 * theta2;
    const T thetad = theta * (T(1) + k1 * theta2 + k2 * theta4);
    *du = u * thetad / r - u;
    *dv = v * thetad / r - v;
  } else {
    *du = T(0);
    *dv = T(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// note: ThinPrismFisheyeCameraModel

std::string ThinPrismFisheyeCameraModel::InitializeParamsInfo() {
  return "fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1";
}

std::vector<size_t> ThinPrismFisheyeCameraModel::InitializeFocalLengthIdxs() {
  return {0, 1};
}

std::vector<size_t>
ThinPrismFisheyeCameraModel::InitializePrincipalPointIdxs() {
  return {2, 3};
}

std::vector<size_t> ThinPrismFisheyeCameraModel::InitializeExtraParamsIdxs() {
  return {4, 5, 6, 7, 8, 9, 10, 11};
}

std::vector<double> ThinPrismFisheyeCameraModel::InitializeParams(
    const double focal_length, const size_t width, const size_t height) {
  return {focal_length,
          focal_length,
          width / 2.0,
          height / 2.0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0};
}

template <typename T>
void ThinPrismFisheyeCameraModel::WorldToImage(const T* params, const T u,
                                               const T v, T* x, T* y) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  const T r = ceres::sqrt(u * u + v * v);

  T uu, vv;
  if (r > T(std::numeric_limits<double>::epsilon())) {
    const T theta = ceres::atan(r);
    uu = theta * u / r;
    vv = theta * v / r;
  } else {
    uu = u;
    vv = v;
  }

  // Distortion
  T du, dv;
  Distortion(&params[4], uu, vv, &du, &dv);
  *x = uu + du;
  *y = vv + dv;

  // Transform to image coordinates
  *x = f1 * *x + c1;
  *y = f2 * *y + c2;
}

template <typename T>
void ThinPrismFisheyeCameraModel::ImageToWorld(const T* params, const T x,
                                               const T y, T* u, T* v) {
  const T f1 = params[0];
  const T f2 = params[1];
  const T c1 = params[2];
  const T c2 = params[3];

  // Lift points to normalized plane
  *u = (x - c1) / f1;
  *v = (y - c2) / f2;

  IterativeUndistortion(&params[4], u, v);

  const T theta = ceres::sqrt(*u * *u + *v * *v);
  const T theta_cos_theta = theta * ceres::cos(theta);
  if (theta_cos_theta > T(std::numeric_limits<double>::epsilon())) {
    const T scale = ceres::sin(theta) / theta_cos_theta;
    *u *= scale;
    *v *= scale;
  }
}

template <typename T>
void ThinPrismFisheyeCameraModel::Distortion(const T* extra_params, const T u,
                                             const T v, T* du, T* dv) {
  const T k1 = extra_params[0];
  const T k2 = extra_params[1];
  const T p1 = extra_params[2];
  const T p2 = extra_params[3];
  const T k3 = extra_params[4];
  const T k4 = extra_params[5];
  const T sx1 = extra_params[6];
  const T sy1 = extra_params[7];

  const T u2 = u * u;
  const T uv = u * v;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T r4 = r2 * r2;
  const T r6 = r4 * r2;
  const T r8 = r6 * r2;
  const T radial = k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
  *du = u * radial + T(2) * p1 * uv + p2 * (r2 + T(2) * u2) + sx1 * r2;
  *dv = v * radial + T(2) * p2 * uv + p1 * (r2 + T(2) * v2) + sy1 * r2;
}

////////////////////////////////////////////////////////////////////////////////

void CameraModelWorldToImage(const int model_id,
                             const std::vector<double>& params, const double u,
                             const double v, double* x, double* y) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                    \
  case CameraModel::kModelId:                             \
    CameraModel::WorldToImage(params.data(), u, v, x, y); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

void CameraModelImageToWorld(const int model_id,
                             const std::vector<double>& params, const double x,
                             const double y, double* u, double* v) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                    \
  case CameraModel::kModelId:                             \
    CameraModel::ImageToWorld(params.data(), x, y, u, v); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }
}

double CameraModelImageToWorldThreshold(const int model_id,
                                        const std::vector<double>& params,
                                        const double threshold) {
  switch (model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                   \
  case CameraModel::kModelId:                                            \
    return CameraModel::ImageToWorldThreshold(params.data(), threshold); \
    break;

    CAMERA_MODEL_SWITCH_CASES

#undef CAMERA_MODEL_CASE
  }

  return -1;
}

}  // namespace colmap

#endif  // COLMAP_SRC_BASE_CAMERA_MODELS_H_
