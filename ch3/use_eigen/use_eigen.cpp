#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

int main(int argc, char** argv) {
    // init log and flags
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::LogToStderr();          // output the info to terminal

    // rotation matrix <---> rotation vector
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
    LOG(INFO) << "rotation matrix = \n" << rotation_vector.matrix();
    rotation_matrix = rotation_vector.toRotationMatrix();
    // rotate the coordinate
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    LOG(INFO) << "(1, 0, 0) after rotation = " << v_rotated.transpose();
    v_rotated = rotation_matrix * v;
    LOG(INFO) << "(1, 0, 0) after rotation = " << v_rotated.transpose();

    //euler angles
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序
    LOG(INFO) << "yaw picth roll = " << euler_angles.transpose();

    // transform matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(1, 3, 4));
    LOG(INFO) << "Transform matrix = \n" << T.matrix();
    Eigen::Vector3d v_transformed = T * v;
    LOG(INFO) << "(1, 0, 0) tranformed = " << v_transformed.transpose();

    // Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    LOG(INFO) << "quaternion = \n" << q.coeffs();       // x, y, z, w
    q = Eigen::Quaterniond(rotation_matrix);
    LOG(INFO) << "quaternion = \n" << q.coeffs();
    v_rotated = q * v;
    LOG(INFO) << "(1, 0, 0) after rotation = " << v_rotated.transpose();

    return 0;
}
