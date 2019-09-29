#ifndef POSE_ESTIMATE_H_
#define POSE_ESTIMATE_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::vector;
using std::cout;
using std::endl;

void pose_estimate(const vector<cv::KeyPoint>& keypoints_1,
                   const vector<cv::KeyPoint>& keypoints_2,
                   const vector<cv::DMatch>& matches,
                   cv::Mat& R, cv::Mat& t);

#endif // POSE_ESTIMATE_H_
