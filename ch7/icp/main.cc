#include <iostream>
#include <vector>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "feature_extraction/extract_feature.h"
#include "utils/utils.h"
#include "svd_icp/svd.h"

int main(int argc, char** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  if(argc != 5) {
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
  LOG(INFO) << "match size = " << matches.size();

  // load depth image
  cv::Mat depth_img1 = cv::imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat depth_img2 = cv::imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat K = (cv::Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  // find keypoints depth
  std::vector<cv::Point3f> pts1_3d;
  std::vector<cv::Point3f> pts2_3d;
  for(const auto& match : matches) {
    ushort d1 = depth_img1.ptr<unsigned short>(int(keypoints_1[match.queryIdx].pt.y))[
        int(keypoints_1[match.queryIdx].pt.x)];
    ushort d2 = depth_img2.ptr<unsigned short>(int(keypoints_2[match.trainIdx].pt.y))[
        int(keypoints_2[match.trainIdx].pt.x)];
    if(d1 == 0 || d2 == 0) continue;

    float dd1 = d1 / 1000.0;
    float dd2 = d2 / 1000.0;
    cv::Point2d pt1 = pixel2cam(keypoints_1[match.queryIdx].pt, K);
    cv::Point2d pt2 = pixel2cam(keypoints_2[match.trainIdx].pt, K);

    pts1_3d.emplace_back(cv::Point3f(pt1.x * dd1, pt1.y * dd1, dd1));
    pts2_3d.emplace_back(cv::Point3f(pt2.x * dd2, pt2.y * dd2, dd2));
  }

  LOG(INFO) << "3d-3d pairs = " << pts1_3d.size();

  // ICP via svd
  cv::Mat R, t;
  svd_icp(pts1_3d, pts2_3d, R, t);

  return 0;
}
