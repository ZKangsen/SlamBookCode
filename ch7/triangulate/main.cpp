#include <iostream>
#include <fstream>
#include <vector>
#include <glog/logging.h>
#include "utils/utils.h"
#include "feature_extraction/extract_feature.h"
#include "pose_estimate/pose_estimate.h"
#include "triangulate/triangulate.h"

using std::vector;

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

  std::vector<cv::Point3d> points;
  triangulate(keypoints_1, keypoints_2, matches, R, t, points);

  // verify the reprojection relation
  cv::Mat K = (cv::Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  std::ofstream fout("reprojection_res.txt");
  for(int i = 0; i < matches.size(); ++i) {
    // first camera frame
    cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
    cv::Point2d pt1_cam_3d = cv::Point2d(points[i].x / points[i].z, points[i].y / points[i].z);

    fout << "point in the first camera: " << pt1_cam << " ";
    fout << "point project from 3D: " << pt1_cam_3d << " d = " << points[i].z << "\n";
//    LOG(INFO) << "point in the first camera: " << pt1_cam;
//    LOG(INFO) << "point project from 3D: " << pt1_cam_3d << " d = " << points[i].z;

    // second camera frame
    cv::Point2d pt2_cam = pixel2cam(keypoints_2[matches[i].trainIdx].pt, K);
    cv::Mat pt2_cam_3d = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
    pt2_cam_3d /= pt2_cam_3d.at<double>(2, 0);
    fout << "point in the second camera: " << pt2_cam << " ";
    fout << "point project from 3D: " << pt2_cam_3d.t() << "\n";
//    LOG(INFO) << "point in the second camera: " << pt2_cam;
//    LOG(INFO) << "point project from 3D: " << pt2_cam_3d.t();
  }

  return 0;
}


