#include "triangulate.h"
#include "../utils/utils.h"

void triangulate(const std::vector<cv::KeyPoint>& keypoints_1,
                 const std::vector<cv::KeyPoint>& keypoints_2,
                 const std::vector<cv::DMatch>& matches,
                 const cv::Mat& R, const cv::Mat& t,
                 std::vector<cv::Point3d>& points) {
  cv::Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
                                          0, 1, 0, 0,
                                          0, 0, 1, 0);
  cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
      R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

  // pixel to cam
  cv::Mat K = (cv::Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  std::vector<cv::Point2d> pts_1, pts_2;
  for(const auto& match : matches) {
    cv::Point2d pt1 = pixel2cam(keypoints_1[match.queryIdx].pt, K);
    cv::Point2d pt2 = pixel2cam(keypoints_2[match.trainIdx].pt, K);
    pts_1.emplace_back(pt1);
    pts_2.emplace_back(pt2);
  }

  // triangulate
  cv::Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // normalized coordinates
  for(int i = 0; i < pts_4d.cols; ++i) {
    cv::Mat pt_4d = pts_4d.col(i);
    pt_4d /= pt_4d.at<double>(3, 0);        // normalize
    cv::Point3d pt_3d(pt_4d.at<double>(0, 0), pt_4d.at<double>(1, 0), pt_4d.at<double>(2, 0));
    points.emplace_back(pt_3d);
  }
}