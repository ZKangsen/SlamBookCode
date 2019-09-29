#include "utils.h"

cv::Point2d pixel2cam(const cv::Point2f& pixel, const cv::Mat& k) {
  cv::Mat pixelMat = (cv::Mat_<double>(3, 1) << pixel.x, pixel.y, 1);
  cv::Mat camMat = k.inv() * pixelMat;
  cv::Point2d ptCam(camMat.at<double>(0, 0) / camMat.at<double>(2, 0),
                    camMat.at<double>(1, 0) / camMat.at<double>(2, 0));
  return ptCam;
}