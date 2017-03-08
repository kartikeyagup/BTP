#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/core/core.hpp>
#include <vector>

struct plane {
  float a, b, c, d;

  plane(){};
  plane(float x1, float x2, float x3, float x4) : a(x1), b(x2), c(x3), d(x4){};

  float norm() { return sqrt(a * a + b * b + c * c); }

  float dotp(plane p) {
    float answer = a * p.a + b * p.b + c * p.c;
    float denom = norm() * p.norm();
    if (denom == 0) {
      return 0;
    } else {
      return answer / denom;
    }
  }
};

void segment_Points(std::vector<cv::Point3f> &inputpoints,
                    std::vector<int> &inliers, plane &p, float distance);

void fit3Planes(std::vector<cv::Point3f> &inputpoints,
                std::vector<cv::Point3f> &plane1,
                std::vector<cv::Point3f> &plane2,
                std::vector<cv::Point3f> &plane3, plane &p1, plane &p2,
                plane &p3, float distance);

#endif
