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
  Eigen::Matrix3d R = Eigen::AngleAxis(M_PI / 3, Eigen::Vector3d(1, 0, 0)).matrix();
  LOG(INFO) << "origin matrix = \n" << R;
  Eigen::Vector3d w(0.01, 0.02, 0.03);



  return 0;
}

