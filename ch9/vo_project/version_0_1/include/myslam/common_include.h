#ifndef MYSLAM_COMMON_INCLUDE_H_
#define MYSLAM_COMMON_INCLUDE_H_

// std
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include <cmath>
//glog
#include <glog/logging.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//sophus
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