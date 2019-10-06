#ifndef MYSLAM_COMMON_INCLUDE_H_
#define MYSLAM_COMMON_INCLUDE_H_

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/so3.h>
#include <sophus/se3.h>

namespace myslam {

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Sophus::SE3;
using Sophus::SO3;
using cv::Mat;

} // namespace myslam

#endif // MYSLAM_COMMON_INCLUDE_H_