#ifndef TRIANGULATE_H_
#define TRIANGULATE_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

void triangulate(const std::vector<cv::KeyPoint>& keypoints_1,
                 const std::vector<cv::KeyPoint>& keypoints_2,
                 const std::vector<cv::DMatch>& matches,
                 const cv::Mat& R, const cv::Mat& t,
                 std::vector<cv::Point3d>& points);

#endif // TRIANGULATE_H_
