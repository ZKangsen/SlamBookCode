#include "extract_feature.h"

void extract_feature(const cv::Mat& image1, const cv::Mat& image2,
                     vector<cv::KeyPoint>& keypoints_1,
                     vector<cv::KeyPoint>& keypoints_2,
                     vector<cv::DMatch>& good_matches) {
  // init paras
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

  std::cout << "min_dist = " << min_dist << std::endl;
  std::cout << "max_dist = " << max_dist << std::endl;

  for(int i = 0; i < descriptors_1.rows; ++i) {
    if(matches[i].distance < std::max(2 * min_dist, 30.0)) {
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
}
