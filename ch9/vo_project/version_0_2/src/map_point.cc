#include "myslam/map_point.h"

namespace myslam {

MapPoint::MapPoint() : id_(-1), pos_(Vector3d(0, 0, 0)), normal_(Vector3d(0, 0, 0)),
                       observed_times_(0), correct_times_(0) {

}

MapPoint::MapPoint(size_t id, Vector3d position, Vector3d normal) : id_(id), pos_(position), normal_(normal),
                                                                    observed_times_(0), correct_times_(0) {

}

MapPoint::MapPointPtr MapPoint::CreateMapPoint() {
  static size_t factory_id = 0;
  return MapPointPtr(new MapPoint(factory_id, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
}

size_t MapPoint::Id() const {
  return id_;
}

} // namespace myslam
