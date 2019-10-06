#include "myslam/frame.h"

namespace myslam {

Frame::Frame() {

}

Frame::Frame(size_t frame_id, double time_stamp, SE3 T_c_w, CamPtr camera, Mat img, Mat depth)
        : frame_id_(frame_id), time_stamp_(time_stamp), T_c_w_(T_c_w),
          camera_(camera), img_(img), depth_(depth) {
  camera_.reset(new Camera(520.9, 521.0, 325.1, 249.7));
}

// 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1

Frame::FramePtr Frame::CreateFrame() {
  std::shared_ptr<Frame> frame_ptr;
  frame_ptr.reset(new Frame(0));
  return frame_ptr;
}

double Frame::FindDepth(const cv::KeyPoint& keypoint) const {
  double d = depth_.ptr<double>(int(keypoint.pt.y))[int(keypoint.pt.x)];
  return d;
}

Vector3d Frame::GetCameraCenter() const {
  return camera_->GetCameraCenter();
}

bool Frame::IsInFrame(const Vector3d& pt_world) const {
  // convert to camera coordinate
  Vector3d pt_camera = camera_->World2Camera(pt_world, T_c_w_);
  Vector2d pt_pixel = camera_->Camera2Pixel(pt_camera);
  double d = depth_.ptr<double>(int(pt_pixel.y()))[int(pt_pixel.x())];
  if(d != pt_world(2)) {
    return false;
  }
  return true;
}

MapPoint::MapPoint() {

}

MapPoint::MapPoint(size_t id, Vector3d position, Vector3d normal) {

}

MapPoint::MapPointPtr MapPoint::CreateMapPoint() {

}

} // namespace myslam
