#include "densehelpers.h"

bool inRange(float x, float limt) {
  return (abs(x)<limt-1);
}

// x+y and x-y lie in the limt and stay positive
bool inConstrainedRange(float x, float y, float limt) {
  return ((x+y<limt-1) && (x-y>=0));
}

float get_score(cv::Mat &img1, cv::Mat &img2, cv::Point2f p1, cv::Point2f p2) {
  int gridsize = 3;
  if (inConstrainedRange(p1.x, gridsize, img1.cols) && inConstrainedRange(p1.y, gridsize, img1.rows)) {
    if (inConstrainedRange(p2.x, gridsize, img1.cols) && inConstrainedRange(p2.y, gridsize, img2.rows)) {
      float answer = 0;
      for (int i=-gridsize; i<=gridsize; i++) {
        for (int j=-gridsize; j<=gridsize; j++) {
          cv::Point2f temp(i,j);
          float diff = img1.at<uchar>(p1+temp) - img2.at<uchar>(p2+temp); 
          answer += diff*diff;
        }
      }
      // answer /= 255.0;
      answer /= (1+gridsize)*(1+gridsize);
      // answer = abs(answer);
      // if (answer < 0) {
      //   answer *= -1;
      // }
      // std::cerr << answer << "\n";
      return answer;
    }
  } 
  return 100000;
}

bool findPoint(cv::Point2f pt, 
  cv::Mat &img1,
  cv::Mat &img2,
  camera_frame_wo_image &frame_1,
  camera_frame_wo_image &frame_2,
  cv::Point2f &location) {
  Eigen::Matrix3f tx;
  tx.setZero();
  cv::Point3f temp1 = frame_2.position - frame_1.position;
  Eigen::Vector3f temp;
  temp(0, 0) = temp1.x;
  temp(1, 0) = temp1.y;
  temp(2, 0) = temp1.z;

  temp = frame_1.rotation*temp;
  Eigen::Matrix3f r = frame_2.rotation * frame_1.rotation.transpose();
  tx(0,1) = - temp(2, 0);
  tx(0,2) =   temp(1, 0);
  tx(1,0) =   temp(2, 0);
  tx(1,2) = - temp(0, 0);
  tx(2,0) = - temp(1, 0);
  tx(2,1) =   temp(0, 0);
  Eigen::Matrix3f prod = tx*r;
  Eigen::Vector3f p1;
  cv::Point2f center(img1.cols/2, img1.rows/2);
  std::vector<cv::Point2f> corners;
  pt -= center;
  p1(0, 0) = pt.x / frame_1.intrinsics.f;
  p1(1, 0) = pt.y / frame_1.intrinsics.f;
  p1(2, 0) = 1;
  Eigen::Vector3f line = prod*p1;
  line(2, 0)*= frame_2.intrinsics.f;

  if (line(0,0)<1e-8) {
    // std::cerr << "Ill conditioned x\n";
    return false;
  }
  if (line(1,0)<1e-8) {
    // std::cerr << "Ill conditioned y\n";
    return false;
  }

  if (inRange((-line(2,0)-line(1,0)*center.y)/line(0,0), center.x)) {
    corners.push_back(cv::Point2f((-line(2,0)-line(1,0)*center.y)/line(0,0), center.y));
  }
  if (inRange((-line(2,0)+line(1,0)*center.y)/line(0,0), center.x)) {
    corners.push_back(cv::Point2f((-line(2,0)+line(1,0)*center.y)/line(0,0), -center.y));
  }
  if (inRange((-line(2,0)-line(0,0)*center.x)/line(1,0), center.y)) {
    corners.push_back(cv::Point2f(center.x, (-line(2,0)-line(0,0)*center.x)/line(1,0)));
  }
  if (inRange((-line(2,0)+line(0,0)*center.x)/line(1,0), center.y)) {
    corners.push_back(cv::Point2f(-center.x, (-line(2,0)-line(0,0)*center.x)/line(1,0)));
  }
  assert(corners.size()<=2);
  if (corners.size()<2) {
    // std::cerr << "Only 1 point in the frame\n";
    return false;
  }

  assert(corners.size()==2);
  for (int i=0; i<corners.size(); i++) {
    corners[i] += center;
  }
  
  float bestsofar = 10000;
  cv::Point found;
  for (int i=0; i<100; i++) {
    for (int j=-3; j<=3; j++) {
      cv::Point2f linept = (i*corners[0] + (100-i)*corners[1])/100;
      linept.y += j;
      // cv::circle(img2, linept, 5, cv::Scalar(255,0,0), -1);
      float score = get_score(img1, img2, pt, linept);
      if (score<bestsofar) {
        // std::cerr << score << "\n";
        bestsofar = score;
        found = linept;
      }
    }
  }

  if (bestsofar>100) {
    // std::cerr << "Could not find a good point: " << bestsofar << "\n";
    return false;
  } else {
    location = found;
    // cv::namedWindow("new", 0);
    // cv::namedWindow("orig", 1);
    // cv::circle(img1, pt+center, 10, cv::Scalar(0), -1);
    // cv::circle(img2, corners[0], 10, cv::Scalar(0,0,255), -1);
    // cv::circle(img2, corners[1], 10, cv::Scalar(0,0,255), -1);
    // cv::circle(img2, found, 10, cv::Scalar(255,0,0), -1);
    // cv::line(img2, corners[0], corners[1], cv::Scalar(0));
    // cv::imshow("orig", img1);
    // cv::imshow("new", img2);
    // while (cv::waitKey(1) != 27) {
    // }

    return true;
  }

  assert(false);
  return true;
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
