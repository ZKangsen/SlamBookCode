#include "myslam/map.h"

namespace myslam {

void Map::InsertMapPoint(const MapPoint::MapPointPtr map_point) {
  if(map_points_.find(map_point->Id()) == map_points_.end()) {
    map_points_.insert(std::make_pair(map_point->Id(), map_point));
  } else {
    map_points_[map_point->Id()] = map_point;
  }
}

void Map::InsertKeyFrame(const Frame::FramePtr frame) {
  LOG(INFO) << "keyframe size = " << keyframes_.size();
  if(keyframes_.find(frame->FrameId()) == keyframes_.end()) {
    keyframes_.insert(std::make_pair(frame->FrameId(), frame));
  } else {
    keyframes_[frame->FrameId()] = frame;
  }
}

} // namespace myslam
