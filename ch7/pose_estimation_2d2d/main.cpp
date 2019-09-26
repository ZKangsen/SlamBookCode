#include <iostream>
#include <fstream>
#include <vector>
#include <glog/logging.h>
#include "feature_extraction/extract_feature.h"
#include "pose_estimate/pose_estimate.h"

using std::vector;

cv::Point2d pixel2cam(cv::Point2f pt, cv::Mat k);

int main(int argc, char** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  if(argc != 3) {
    LOG(ERROR) << "please input two images correctly";
    return -1;
  }

  // read image
  cv::Mat image1, image2;
  image1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  image2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

  // init paras
  vector<cv::KeyPoint> keypoints_1, keypoints_2;
  vector<cv::DMatch> matches;
  extract_feature(image1, image2, keypoints_1, keypoints_2, matches);
  LOG(INFO) << "matches size = " << matches.size();

  cv::Mat R, t;
  pose_estimate(keypoints_1, keypoints_2, matches, R, t);

  // E = t^R*scale
  cv::Mat t_x = (cv::Mat_<double> (3, 3) << 0.0, -t.at<double>(2, 0), t.at<double>(1, 0),
                                            t.at<double>(2, 0), 0.0, -t.at<double>(0, 0),
                                            -t.at<double>(1, 0), t.at<double>(0, 0), 0.0);
  LOG(INFO) << "t^R = " << t_x * R;

  // 验证对极约束
  std::ofstream fout("pole_constraints_residuals.txt");
  cv::Mat K = (cv::Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  for(auto& match : matches) {
    cv::Point2d pt1 = pixel2cam(keypoints_1[match.queryIdx].pt, K);
    cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    cv::Point2d pt2 = pixel2cam(keypoints_2[match.trainIdx].pt, K);
    cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    cv::Mat d = y2.t() * t_x * R * y1;
    fout << "residual = " << d << "\n";
  }
  fout.flush();
  fout.close();
  return 0;
}

cv::Point2d pixel2cam(cv::Point2f pt, cv::Mat k) {
  cv::Mat ptMat = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1);
  cv::Mat camMat = k.inv() * ptMat;
  cv::Point2d ptCam(camMat.at<double>(0, 0) / camMat.at<double>(2, 0),
      camMat.at<double>(1, 0) / camMat.at<double>(2, 0));
  return ptCam;
}
