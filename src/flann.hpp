#ifndef _FLANN_H
#define _FLANN_H
#include <opencv2/core.hpp>
std::vector<cv::DMatch> flann(cv::Mat descriptors1, cv::Mat descriptors2, double threshold);
#endif
