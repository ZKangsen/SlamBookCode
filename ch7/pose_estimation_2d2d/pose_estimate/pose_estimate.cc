#include "pose_estimate.h"
#include <opencv2/calib3d.hpp>

void pose_estimate(const vector<cv::KeyPoint>& keypoints_1,
                   const vector<cv::KeyPoint>& keypoints_2,
                   const vector<cv::DMatch>& matches,
                   cv::Mat& R, cv::Mat& t) {
  cv::Mat K = (cv::Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  vector<cv::Point2f> points1;
  vector<cv::Point2f> points2;

  for(int i = 0; i < matches.size(); ++i) {
    points1.emplace_back(keypoints_1[matches[i].queryIdx].pt);
    points2.emplace_back(keypoints_2[matches[i].trainIdx].pt);
  }

  // 基础矩阵
  cv::Mat fundamentalMat;
  fundamentalMat = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
  cout << "fundamentalMat = \n" << fundamentalMat << endl;

  // 本质矩阵
  cv::Mat essentialMat;
  essentialMat = cv::findEssentialMat(points1, points2, K, cv::RANSAC);
  cout << "essentialMat = \n" << essentialMat << endl;

  // 单应矩阵
  cv::Mat homographyMat;
  homographyMat = cv::findHomography(points1, points2, cv::RANSAC, 3, cv::noArray(), 2000, 0.995);
  cout << "homographyMat = \n" << homographyMat << endl;

  // compute R, t
  cv::recoverPose(essentialMat, points1, points2, K, R, t);
  cout << "R = \n" << R << endl;
  cout << "t = \n" << t << endl;
}