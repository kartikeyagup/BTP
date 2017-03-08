#ifndef INDOORHELPERS_H
#define INDOORHELPERS_H

#include "nvmhelpers.h"
#include "pointcloud.h"

struct corridor {
  plane roof;
  plane left_wall;
  plane right_wall;
  std::vector<cv::Point3f> points_roof;
  std::vector<cv::Point3f> points_left_wall;
  std::vector<cv::Point3f> points_right_wall;
  std::vector<cv::Point3f> trajectory;

  corridor(){};
  corridor(nvm_file &f, int st, int end, std::string testpath) {
    std::vector<cv::Point3f> reqdpts, plane1, plane2, plane3;
    plane p1, p2, p3;
    f.get_points(st, end, reqdpts, trajectory);
    fit3Planes(reqdpts, plane1, plane2, plane3, p1, p2, p3, f.mdistance());
    std::cout << "Distance kept is " << f.mdistance() << "\n";
    float dp12 = fabs(p1.dotp(p2));
    float dp13 = fabs(p1.dotp(p3));
    float dp23 = fabs(p2.dotp(p3));
    std::cout << "Dot products " << dp12 << "\t" << dp13 << "\t" << dp23
              << "\n";
    if (dp12 > dp23 and dp12 > dp13) {
      // 3 is the roof
      roof = p3;
      points_roof = plane3;
      // TODO
      left_wall = p1;
      right_wall = p2;
      points_left_wall = plane1;
      points_right_wall = plane2;
    } else if (dp13 > dp23 and dp13 > dp12) {
      // 2 is the roof
      roof = p2;
      points_roof = plane2;
      // TODO
      left_wall = p1;
      right_wall = p3;
      points_left_wall = plane1;
      points_right_wall = plane3;
    } else {
      // 1 is the roof
      roof = p1;
      points_roof = plane1;
      // TODO
      left_wall = p3;
      right_wall = p2;
      points_left_wall = plane3;
      points_right_wall = plane2;
    }
    // Run optimisation pipeline
    nvm_file temp;
    for (int i = 0; i < points_roof.size(); i++) {
      temp.corr_data.push_back(Corr3D(points_roof[i], cv::Point3i(255, 0, 0)));
    }
    for (int i = 0; i < points_left_wall.size(); i++) {
      temp.corr_data.push_back(
          Corr3D(points_left_wall[i], cv::Point3i(0, 255, 0)));
    }
    for (int i = 0; i < points_right_wall.size(); i++) {
      temp.corr_data.push_back(
          Corr3D(points_right_wall[i], cv::Point3i(0, 0, 255)));
    }
    temp.save_ply_file(testpath);
  }
};

#endif
