#ifndef BUNDLE_ADJUSTMENT_H_
#define BUNDLE_ADJUSTMENT_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

void BundleAdjustment(const std::vector<cv::Point3f>& pts_3d,
                      const std::vector<cv::Point2f>& pts_2d,
                      const cv::Mat& K,
                      const cv::Mat& R, const cv::Mat& t);

#endif // BUNDLE_ADJUSTMENT_H_