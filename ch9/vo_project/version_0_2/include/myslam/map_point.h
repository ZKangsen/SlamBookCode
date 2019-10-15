#ifndef MYSLAM_MAP_POINT_H_
#define MYSLAM_MAP_POINT_H_

#include "common_include.h"

namespace myslam {

class MapPoint {
  public:
    typedef std::shared_ptr<MapPoint> MapPointPtr;
    typedef std::shared_ptr<const MapPoint> MapPointConstPtr;

    MapPoint();
    MapPoint(size_t id, Vector3d position, Vector3d normal);
    static MapPointPtr CreateMapPoint();
    size_t Id() const;

  private:
    size_t id_;         // id
    Vector3d pos_;     // pose in world
    Vector3d normal_;   // normal of viewing direction
    Mat descriptor_;    // descriptor
    int observed_times_; // being observed by feature matching algo
    int correct_times_;  // being an inlier in pose estimation
}; // class MapPoint

} // namespace myslam

#endif // MYSLAM_MAP_POINT_H_
