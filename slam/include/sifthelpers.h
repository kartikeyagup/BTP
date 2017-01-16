#ifndef SIFT_HELPERS_H
#define SIFT_HELPERS_H

#include <cstddef>
#include <SiftGPU/SiftGPU.h>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <opencv2/core/core.hpp>

std::vector<std::pair<cv::Point2f, cv::Point2f> > RunSift(std::string f1, std::string f2, cv::Point2f center);

#endif
