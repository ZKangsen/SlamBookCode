#include "myslam/frame.h"

namespace myslam {

Frame::Frame() : frame_id_(-1), time_stamp_(-1), camera_(nullptr) {

}

Frame::Frame(size_t frame_id, double time_stamp, SE3 T_c_w, CamPtr camera, Mat img, Mat depth)
        : frame_id_(frame_id), time_stamp_(time_stamp), T_c_w_(T_c_w),
          camera_(camera), img_(img), depth_(depth) {

}

Frame::FramePtr Frame::CreateFrame() {
  static size_t factory_id = 0;
  return Frame::FramePtr(new Frame(factory_id++));
}

double Frame::FindDepth(const cv::KeyPoint& keypoint) const {
  int x = cvRound(keypoint.pt.x);
  int y = cvRound(keypoint.pt.y);
  ushort d = depth_.ptr<ushort>(y)[x];
  if(d != 0) {
    return double(d) / camera_->GetDepthScale();
  } else {
    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, -1, 0, 1};
    for(int i = 0; i < 4; ++i) {
      d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
      if(d != 0) {
        return double(d) / camera_->GetDepthScale();
      }
    }
  }
  return -1.0;
}

/** T_c_w is extrinsic, T_c_w = W_T_C, T_c_w * Pw = W_T_C * Pw = Pc, (convert world to camera)
 *  T_w_c = T_c_w.inverse(), T_w_c = C_T_W, T_w_c is camera pose in world,
 *  if center(0, 0) in camera, then center in world is center' = R * center + t = t
 */
Vector3d Frame::GetCameraCenter() const {
  return T_c_w_.inverse().translation();
}

bool Frame::IsInFrame(const Vector3d& pt_world) const {
  // convert to camera coordinate
  Vector3d pt_camera = camera_->World2Camera(pt_world, T_c_w_);
  if(pt_camera.z() < 0) {
    return false;
  }
  Vector2d pt_pixel = camera_->World2Pixel(pt_world, T_c_w_);
  return pt_pixel.x() > 0 &&
         pt_pixel.y() > 0 &&
         pt_pixel.x() < img_.cols &&
         pt_pixel.y() < img_.rows;
}

size_t Frame::FrameId() const {
  return frame_id_;
}

} // namespace myslam
