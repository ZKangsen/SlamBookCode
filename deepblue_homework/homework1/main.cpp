#include <iostream>
#include <cmath>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.h>

int main(int argc, char** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  // update rotation
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
  LOG(INFO) << "origin R = \n" << R;
  Eigen::Vector3d w(0.01, 0.02, 0.03);
  Sophus::SO3 SO3_R(R);
  Sophus::SO3 SO3_updated = SO3_R * Sophus::SO3::exp(w);
  Eigen::Matrix3d R_updated = SO3_updated.matrix();
  LOG(INFO) << "after R update, R = \n" << R_updated;

  // use q
  Eigen::Quaterniond q = Eigen::Quaterniond(R);
  Eigen::Quaterniond w_q(1, w.x() / 2, w.y() / 2, w.z() / 2);
  Eigen::Quaterniond q_updated = q * w_q;
  LOG(INFO) << "after q update, R = \n" << q_updated.toRotationMatrix();


  return 0;
}

