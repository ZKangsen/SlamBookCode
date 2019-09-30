#ifndef EXTRATE_FEATURE_H_
#define EXTRATE_FEATURE_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::vector;

void extract_feature(const cv::Mat& image1, const cv::Mat& image2,
                     vector<cv::KeyPoint>& keypoints_1,
                     vector<cv::KeyPoint>& keypoints_2,
                     vector<cv::DMatch>& good_matches);

#endif // EXTRATE_FEATURE_H_