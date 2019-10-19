#include <iostream>
#include <cmath>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.h>

Eigen::Matrix3d UpdateRotationMatrix(const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& w,
                               bool use_sophus = false);

Eigen::Quaterniond UpdateQuaterniond(const Eigen::Quaterniond& q,
                                     const Eigen::Vector3d& w);

int main(int argc, char** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  // Test the difference of rotation updates between matrix and quaterniond
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
  Eigen::Quaterniond Q(R);
  Eigen::Vector3d w(0.01, 0.02, 0.03);
  Eigen::Matrix3d R_updated = UpdateRotationMatrix(R, w);
  Eigen::Quaterniond Q_updated = UpdateQuaterniond(Q, w);
  LOG(INFO) << "origin_R = \n" << R;
  LOG(INFO) << "After updating with rotation matrix, R_updated = \n" << R_updated;
  LOG(INFO) << "After updating with quaternion, R_updated = \n" << Q_updated.toRotationMatrix();
  return 0;
}

Eigen::Matrix3d UpdateRotationMatrix(const Eigen::Matrix3d& R,
                                     const Eigen::Vector3d& w,
                                     bool use_sophus) {
  Eigen::Matrix3d R_updated(R);
  if(use_sophus) {
    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_updated = SO3_R * Sophus::SO3::exp(w);
    R_updated = SO3_updated.matrix();
  } else {
    double theta = w.norm();
    Eigen::Vector3d n = w / theta;
    Eigen::Matrix3d nnT = n * n.transpose();
    Eigen::Matrix3d n_hat;
    n_hat << 0, -n.z(), n.y(),
             n.z(), 0, -n.x(),
             -n.y(), n.x(), 0;
    Eigen::Matrix3d update_R =
        cos(theta) * Eigen::Matrix3d::Identity() + (1 - cos(theta)) * nnT + sin(theta) * n_hat;
    R_updated = R * update_R;
  }
  return R_updated;
}

Eigen::Quaterniond UpdateQuaterniond(const Eigen::Quaterniond& q, const Eigen::Vector3d& w) {
  Eigen::Quaterniond q_w(1, w.x() / 2, w.y() / 2, w.z() / 2);
  q_w.normalize();
  return q * q_w;
}
