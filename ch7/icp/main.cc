#include <iostream>
#include <vector>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "feature_extraction/extract_feature.h"
#include "utils/utils.h"

#define _use_BA_

int main(int argc, char** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  if(argc != 4) {
    LOG(INFO) << "Input paras is not correct!!!";
    return -1;
  }

  cv::Mat image1, image2;
  image1 = cv::imread(argv[1]);
  image2 = cv::imread(argv[2]);
  std::vector<cv::KeyPoint> keypoints_1;
  std::vector<cv::KeyPoint> keypoints_2;
  std::vector<cv::DMatch> matches;
  extract_feature(image1, image2, keypoints_1, keypoints_2, matches);

  // load depth image
  cv::Mat depth_img = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat K = (cv::Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point2f> pts_2d;
  for(const auto& match : matches) {
    ushort d = depth_img.ptr<unsigned short>(int(keypoints_1[match.queryIdx].pt.y))[
        int(keypoints_1[match.queryIdx].pt.x)];
    if(d == 0) continue;
    float dd = d / 1000.0;

    cv::Point2d pt = pixel2cam(keypoints_1[match.queryIdx].pt, K);
    pts_3d.emplace_back(cv::Point3f(pt.x * dd, pt.y * dd, dd));
    pts_2d.emplace_back(keypoints_2[match.trainIdx].pt);
  }

  LOG(INFO) << "3d-2d pairs = " << pts_3d.size();

  cv::Mat r, t;
  cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false, cv::SOLVEPNP_EPNP);
  cv::Mat R;
  cv::Rodrigues(r, R);

  LOG(INFO) << "before BA, R = \n" << R;
  LOG(INFO) << "before BA, t = " << t.t();

  // BA optimize
#ifdef _use_BA_
BundleAdjustment(pts_3d, pts_2d, K, R, t);
#endif

  return 0;
}
