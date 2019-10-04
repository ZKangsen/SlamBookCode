#ifndef SVD_ICP_H_
#define SVD_ICP_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

void svd_icp(const std::vector<cv::Point3f>& pts1,
             const std::vector<cv::Point3f>& pts2,
             cv::Mat& R, cv::Mat& t);

#endif // SVD_ICP_H_