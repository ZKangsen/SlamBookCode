#ifndef MYSLAM_MAP_H_
#define MYSLAM_MAP_H_

#include "frame.h"

namespace myslam {

class Map {
  public:
    Map() {}
    void InsertMapPoint(const MapPoint::MapPointPtr map_point);
    void InsertKeyFrame(const Frame::FramePtr frame);

  private:
    std::unordered_map<size_t, MapPoint::MapPointPtr> map_points_;
    std::unordered_map<size_t, Frame::FramePtr> keyframes_;
}; // class Map

} // namespace myslam

#endif //MYSLAM_MAP_H_
