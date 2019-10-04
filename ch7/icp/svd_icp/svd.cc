#include "svd.h"
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

void svd_icp(const std::vector<cv::Point3f>& pts1,
             const std::vector<cv::Point3f>& pts2,
             cv::Mat& R, cv::Mat& t) {
  // compute center of mass
  cv::Point3f pt1, pt2;
  int points_num = pts1.size();
  for(int i = 0; i < points_num; ++i) {
    pt1 += pts1[i];
    pt2 += pts2[i];
  }
  pt1 /= points_num;
  pt2 /= points_num;

  // subtract center of mass
  std::vector<cv::Point3f> q1(points_num), q2(points_num);
  for(int i = 0; i < points_num; ++i) {
    q1[i] = pts1[i] - pt1;
    q2[i] = pts2[i] - pt2;
  }

  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for(int i = 0; i < points_num; ++i) {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }

  LOG(INFO) << "W = \n" << W;

  // svd on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d R_;
  Eigen::Vector3d t_;
  R_ = U * (V.transpose());
  t_ = Eigen::Vector3d(pt1.x, pt1.y, pt1.z) - R_ * Eigen::Vector3d(pt2.x, pt2.y, pt2.z);

  LOG(INFO) << "R = \n" << R_;
  LOG(INFO) << "t = " << t_.transpose();

  LOG(INFO) << "R_inv = \n" << R_.inverse();
  LOG(INFO) << "t_inv = " << (-R_.inverse() * t_).transpose();
}