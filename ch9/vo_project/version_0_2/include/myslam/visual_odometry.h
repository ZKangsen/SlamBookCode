#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_

#include "map.h"
#include "config.h"

namespace myslam {

class VisualOdometry {
  public:
    typedef std::shared_ptr<VisualOdometry> VOPtr;
    VisualOdometry() {}
    ~VisualOdometry() {}
    bool AddFrame(Frame::FramePtr frame);

  private:
    void ExtractKeyPoints();
    void ComputeDescriptors();
    void FeatureMatching();
    void PoseEstimationPnp();
    void SetRef3DPoint();

    void AddKeyFrame(Frame::FramePtr frame);
    bool CheckEstimatedPose();
    bool CheckKeyFrame();

  private:
    enum VOState {Initializing = -1, Ok = 0, Lost};
    VOState state_;
    // map and frame
    Map::MapPtr map_;
    Frame::FramePtr cur_;
    Frame::FramePtr ref_;

    // feature extract
    cv::Ptr<cv::ORB> orb_;
    std::vector<cv::KeyPoint> keypoint_cur_;
    std::vector<cv::Point3f> pts_3d_ref_;
    Mat descriptors_cur_;
    Mat descriptors_ref_;
    std::vector<cv::DMatch> feature_matches_;

    // pose estimate
    SE3 T_c_r_estimate_;    // the estimate pose of current frame
    int num_inliers_;       // number of inlier features in icp
    int num_lost_;          // number of vo lost times

    // paras
    int num_of_features_;   // number of features
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;     // number of pyramid levels
    float match_ratio_;     // ration for selecting good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // min number of inliers

    double key_frame_min_rot_;  // min rotation of two key-frames
    double key_frame_min_trnas_;// min translation of two key-frames
}; // class VisualOdometry

} // namespace myslam

#endif // VISUAL_ODOMETRY_H_