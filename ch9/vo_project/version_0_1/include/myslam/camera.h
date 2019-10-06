#ifndef MYSLAM_CAMERA_H_
#define MYSLAM_CAMERA_H_

#include "myslam/common_include.h"

namespace myslam {

class Camera {
  public:
    Camera();
    Camera(double fx, double fy, double cx, double cy, double depth_scale = 0) :
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {
      intrinsic_ << fx, 0, cx,
                    0, fy, cy,
                    0, 0,  1;
    }

    // coordinate transformation, T_c_w is camera extrinsic
    Vector3d World2Camera(const Vector3d& p_w, const SE3& T_c_w) const;
    Vector3d Camera2World(const Vector3d& p_c, const SE3& T_c_w) const;
    Vector2d Camera2Pixel(const Vector3d& p_c) const;
    Vector3d Pixel2Camera(const Vector2d& p_pix, double depth = 1) const;
    Vector2d World2Pixel(const Vector3d& p_w, const SE3& T_c_w) const;
    Vector3d Pixel2World(const Vector2d& p_pix, const SE3& T_c_w, double depth = 1) const;

    Vector3d GetCameraCenter() const;

  private:
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double depth_scale_;
    Matrix3d intrinsic_;
  }; // class Camera

  typedef std::shared_ptr<Camera> CamPtr;
  typedef std::shared_ptr<const Camera> CamConstPtr;

} // namespace myslam

#endif // MYSLAM_CAMERA_H_
