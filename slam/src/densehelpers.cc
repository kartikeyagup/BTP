#include "densehelpers.h"

bool findPoint(cv::Point2f pt, 
  cv::Mat &img1,
  cv::Mat &img2,
  camera_frame_wo_image &frame_1,
  camera_frame_wo_image &frame_2,
  cv::Point2f &location) {
  return false;
}

cv::Point3i findColor(cv::Mat &img, cv::Point2f pt) {
  cv::Point3i answer;
  // Convert point to normal from center subtracted
  cv::Vec3b color = img.at<cv::Vec3b>(cv::Point(pt.x + img.cols/2, pt.y + img.rows/2));
  answer.x = color[0];
  answer.y = color[1];
  answer.z = color[2];
  // Find color
  return answer;
}
