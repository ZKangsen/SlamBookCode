#include <iostream>
#include <vector>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char** argv) {
  // init log
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  if(argc != 3) {
    LOG(ERROR) << "please input two images correctly";
    return -1;
  }

  // read image
  cv::Mat image1, image2;
  image1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  image2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

  // init paras
  vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Mat descriptors_1, descriptors_2;
  cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);

  // detect key points
  orb->detect(image1, keypoints_1);
  orb->detect(image2, keypoints_2);

  // compute descriptors
  orb->compute(image1, keypoints_1, descriptors_1);
  orb->compute(image2, keypoints_2, descriptors_2);

  // draw key points of image1
  cv::Mat outimage1;
  cv::drawKeypoints(image1, keypoints_1, outimage1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  cv::imshow("ORB feature", outimage1);

  // match descriptors
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(descriptors_1, descriptors_2, matches);

  // matches filter
  double min_dist = 10000, max_dist = 0;
  for(int i = 0; i < descriptors_1.rows; ++i) {
    double dist = matches[i].distance;
    if(dist < min_dist) {
      min_dist = matches[i].distance;
    }
    if(dist > max_dist) {
      max_dist = matches[i].distance;
    }
  }

  LOG(INFO) << "max distance = " << max_dist;
  LOG(INFO) << "min distance = " << min_dist;

  vector<cv::DMatch> good_matches;
  for(int i = 0; i < descriptors_1.rows; ++i) {
    if(matches[i].distance < max(2 * min_dist, 30.0)) {
      good_matches.emplace_back(matches[i]);
    }
  }

  cv::Mat origin_match;
  cv::Mat good_match;
  cv::drawMatches(image1, keypoints_1, image2, keypoints_2, matches, origin_match);
  cv::drawMatches(image1, keypoints_1, image2, keypoints_2, good_matches, good_match);
  cv::imshow("origin match", origin_match);
  cv::imshow("good match", good_match);
  cv::waitKey(0);

  return 0;
}
