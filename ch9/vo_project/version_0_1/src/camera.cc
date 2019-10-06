#include "myslam/camera.h"

namespace myslam {

Camera::Camera() {

}

Vector3d Camera::World2Camera(const Vector3d& p_w, const SE3& T_c_w) const {
  return (T_c_w * p_w);
}

Vector3d Camera::Camera2World(const Vector3d& p_c, const SE3& T_c_w) const {
  return (T_c_w.inverse() * p_c);
}

Vector2d Camera::Camera2Pixel(const Vector3d& p_c) const {
//  Vector3d pixel = intrinsic_ * p_c;
//  pixel /= pixel(2);
//  return Vector2d(pixel.x(), pixel.y());
  double u = (fx_ / p_c(2)) * p_c(0) + cx_;
  double v = (fy_ / p_c(2)) * p_c(1) + cy_;
  return Vector2d(u, v);
}

Vector3d Camera::Pixel2Camera(const Vector2d& p_pix, double depth) const {
  double x = (p_pix(0) - cx_) / fx_;
  double y = (p_pix(1) - cy_) / fy_;
  double z = depth;
  return Vector3d(x * z, y * z, z);
}

Vector2d Camera::World2Pixel(const Vector3d& p_w, const SE3& T_c_w) const {
  return Camera2Pixel(World2Camera(p_w, T_c_w));
}

Vector3d Camera::Pixel2World(const Vector2d& p_pix, const SE3& T_c_w, double depth) const {
  return Camera2World(Pixel2Camera(p_pix, depth), T_c_w);
}

Vector3d Camera::GetCameraCenter() const {
  return Vector3d(cx_, cy_, 0);
}

double Camera::GetDepthScale() const {
  return depth_scale_;
}

} // namespace myslam
