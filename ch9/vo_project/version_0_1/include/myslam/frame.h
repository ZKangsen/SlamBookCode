#ifndef MYSLAM_FRAME_H_
#define MYSLAM_FRAME_H_

#include "myslam/camera.h"

namespace myslam {

class Frame {
  public:
    typedef std::shared_ptr<Frame> FramePtr;
    typedef std::shared_ptr<const Frame> FrameConstPtr;
    Frame();
    Frame(size_t frame_id, double time_stamp = 0.0, SE3 T_c_w = SE3(),
          CamPtr camera = nullptr, Mat img = Mat(), Mat depth = Mat());

    ~Frame() {}

    // factory function
    static FramePtr CreateFrame();

    // find the depth of keypoint
    double FindDepth(const cv::KeyPoint &keypoint) const;

    // get camera center
    Vector3d GetCameraCenter() const;

    // check if the point is in the frame
    bool IsInFrame(const Vector3d &pt_world) const;

  private:
    size_t frame_id_;
    double time_stamp_;
    SE3 T_c_w_;
    CamPtr camera_;
    Mat img_;
    Mat depth_;
}; // class Frame

class MapPoint {
  public:
    typedef std::shared_ptr<MapPoint> MapPointPtr;
    typedef std::shared_ptr<const MapPoint> MapPointConstPtr;

    MapPoint();
    MapPoint(size_t id, Vector3d position, Vector3d normal);
    static MapPointPtr CreateMapPoint();

  private:
    size_t id_;         // id
    Vector3d pos_;     // pose in world
    Vector3d normal_;   // normal of viewing direction
    Mat descriptor_;    // descriptor
    int observed_times_; // being observed by feature matching algo
    int correct_times_;  // being an inlier in pose estimation
};

} // namespace myslam

#endif // MYSLAM_FRAME_H_
