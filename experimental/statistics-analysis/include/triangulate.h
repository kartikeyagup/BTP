#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include "common.h"

std::vector<std::vector<cv::Point3f> > detect_triangulate(
    std::vector<camera_frame> camera_frames);

void ConvertPoint(triangulation_bundle &bundle);

cv::Point3f Triangulate(std::vector<triangulation_bundle> &input);

std::vector<std::vector<cv::Point2f> > detect(cv::Mat &image);

#endif
