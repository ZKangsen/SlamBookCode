#include <iostream>
#include <cmath>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.h>
#include <sophus/se3.h>

int main(int argc, char ** argv) {
    // init glog
    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();
    // init SO3
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_v(0, 0, M_PI / 2);
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);

    LOG(INFO) << "SO3 from matrix = \n" << SO3_R;
    LOG(INFO) << "SO3 from vector = \n" << SO3_v;
    LOG(INFO) << "SO3 from quaternion = \n" << SO3_q;

    // SO3 convert to so3
    Eigen::Vector3d so3 = SO3_R.log();
    LOG(INFO) << "so3 = " << so3.transpose();
    // so3 convert to Antisymmetric matrix
    LOG(INFO) << "so3 hat = \n" << Sophus::SO3::hat(so3);
    LOG(INFO) << "so3 hat vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose();

    // add disturbance
    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
    LOG(INFO) << "SO3_updated = \n" << SO3_updated;

    // SE3, similar to SO3
    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3 SE3_Rt(R, t);
    Sophus::SE3 SE3_qt(q, t);
    LOG(INFO) << "SE3, from rotation: \n" << SE3_Rt;
    LOG(INFO) << "SE3, from quaternion: \n" << SE3_qt;

    // SE3 convert to se3
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    LOG(INFO) << "se3 = " << se3.transpose();
    LOG(INFO) << "se3 hat = \n" << Sophus::SE3::hat(se3);
    LOG(INFO) << "se3 hat vee = " << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose();

    // add disturbance
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    LOG(INFO) << "SE3_updated = \n" << SE3_updated.matrix();

    return 0;
}