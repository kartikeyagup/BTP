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

  void WritePly(std::string path) {
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
    for (int i = 0; i < trajectory.size(); i++) {
      temp.corr_data.push_back(
          Corr3D(trajectory[i], cv::Point3i(255, 255, 255)));
    }
    temp.save_ply_file(path);
  }

  corridor(){};
  corridor(nvm_file f, int st, int end, std::string testpath) {
    std::vector<cv::Point3f> reqdpts, plane1, plane2, plane3;
    plane p1, p2, p3;
    f.reset_origin(st);
    f.get_points(st, end, reqdpts, trajectory);
    fit3Planes(reqdpts, plane1, plane2, plane3, p1, p2, p3, f.mdistance());
    float dp12 = fabs(p1.dotp(p2));
    float dp13 = fabs(p1.dotp(p3));
    float dp23 = fabs(p2.dotp(p3));
    if (dp12 > dp23 and dp12 > dp13) {
      // 3 is the roof
      roof = p3;
      points_roof = plane3;
      if (p1.d / p1.a > 0.0) {
        left_wall = p1;
        right_wall = p2;
        points_left_wall = plane1;
        points_right_wall = plane2;
      } else {
        left_wall = p2;
        right_wall = p1;
        points_left_wall = plane2;
        points_right_wall = plane1;
      }
    } else if (dp13 > dp23 and dp13 > dp12) {
      // 2 is the roof
      roof = p2;
      points_roof = plane2;
      if (p1.d / p1.a > 0.0) {
        left_wall = p1;
        right_wall = p3;
        points_left_wall = plane1;
        points_right_wall = plane3;
      } else {
        left_wall = p3;
        right_wall = p1;
        points_left_wall = plane3;
        points_right_wall = plane1;
      }
    } else {
      // 1 is the roof
      roof = p1;
      points_roof = plane1;
      if (p2.d / p2.a > 0.0) {
        left_wall = p2;
        right_wall = p3;
        points_left_wall = plane2;
        points_right_wall = plane3;
      } else {
        left_wall = p3;
        right_wall = p2;
        points_left_wall = plane3;
        points_right_wall = plane2;
      }
    }
    // TODO: Run optimisation pipeline
    std::cout << roof << "\n";
    std::cout << right_wall << "\n";
    std::cout << left_wall << "\n";
    WritePly(testpath);
  }

  float get_width() {
    float answer = fabs(right_wall.d - left_wall.d);
    return answer / sqrt(left_wall.a * left_wall.a + left_wall.b * left_wall.b +
                         left_wall.c * left_wall.c);
  }

  void scale(float f) {
    left_wall.d *= f;
    right_wall.d *= f;
    roof.d *= f;
    for (int i = 0; i < points_roof.size(); i++) {
      points_roof[i] *= f;
    }
    for (int i = 0; i < points_left_wall.size(); i++) {
      points_left_wall[i] *= f;
    }
    for (int i = 0; i < points_right_wall.size(); i++) {
      points_right_wall[i] *= f;
    }
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i] *= f;
    }
  }

  void shift(cv::Point3f s) {
    roof.shift(s);
    left_wall.shift(s);
    right_wall.shift(s);

    for (int i = 0; i < points_roof.size(); i++) {
      points_roof[i] += s;
    }
    for (int i = 0; i < points_left_wall.size(); i++) {
      points_left_wall[i] += s;
    }
    for (int i = 0; i < points_right_wall.size(); i++) {
      points_right_wall[i] += s;
    }
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i] += s;
    }
  }

  static cv::Point3f rotTransform(Eigen::Matrix3f& r, cv::Point3f p) {
    cv::Point3f res;
    res.x = r(0, 0) * p.x + r(0, 1) * p.y + r(0, 2) * p.z;
    res.y = r(1, 0) * p.x + r(1, 1) * p.y + r(1, 2) * p.z;
    res.z = r(2, 0) * p.x + r(2, 1) * p.y + r(2, 2) * p.z;
    return res;
  }

  void rotate(Eigen::Matrix3f rot) {
    roof.rotate(rot);
    left_wall.rotate(rot);
    right_wall.rotate(rot);

    for (int i = 0; i < points_roof.size(); i++) {
      points_roof[i] = rotTransform(rot, points_roof[i]);
    }
    for (int i = 0; i < points_left_wall.size(); i++) {
      points_left_wall[i] = rotTransform(rot, points_left_wall[i]);
    }
    for (int i = 0; i < points_right_wall.size(); i++) {
      points_right_wall[i] = rotTransform(rot, points_right_wall[i]);
    }
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i] = rotTransform(rot, trajectory[i]);
    }
  }

  Eigen::Vector3f GetDirection() {
    Eigen::Vector3f answer;
    answer(0, 0) = trajectory[trajectory.size() - 1].x - trajectory[0].x;
    answer(1, 0) = trajectory[trajectory.size() - 1].y - trajectory[0].y;
    answer(2, 0) = trajectory[trajectory.size() - 1].z - trajectory[0].z;
    answer.normalize();
    return answer;
  }

  Eigen::Vector3f GetAxis() {
    Eigen::Vector3f n1 = roof.GetNormal();
    Eigen::Vector3f n2 = left_wall.GetNormal();
    Eigen::Vector3f a = n1.cross(n2);
    a.normalize();
    Eigen::Vector3f rough = GetDirection();
    float dotp = rough.dot(a);
    if (dotp > 0) {
      return a;
    } else {
      return -a;
    }
  }
};

corridor merge_corridor(corridor c1, corridor c2) {
  corridor answer(c1);
  std::cout << "Left wall " << c1.left_wall.dotp(c2.left_wall) << "\n";
  std::cout << "Right wall " << c1.right_wall.dotp(c2.right_wall) << "\n";
  std::cout << "Roof " << c1.roof.dotp(c2.roof) << "\n";
  std::cout << c1.get_width() << ", " << c2.get_width() << "\n";
  // Scaling
  c2.scale(c1.get_width() / c2.get_width());
  // Rotation
  Eigen::Vector3f axis1, axis2;
  axis1 = c1.GetAxis();
  axis2 = c2.GetAxis();
  std::cout << "Axis 1" << axis1 << "\n";
  std::cout << "Axis 2" << axis2 << "\n";
  Eigen::Vector3f v = axis2.cross(axis1);
  float con = axis2.dot(axis1);
  Eigen::Matrix3f vx;
  vx(0, 0) = 0;
  vx(0, 1) = -v(2, 0);
  vx(0, 2) = v(1, 0);
  vx(1, 0) = v(2, 0);
  vx(1, 1) = 0;
  vx(1, 2) = -v(0, 0);
  vx(2, 0) = -v(1, 0);
  vx(2, 1) = v(0, 0);
  vx(2, 2) = 0;
  Eigen::Matrix3f rot;
  rot.setZero();
  rot(0, 0) = 1.0;
  rot(1, 1) = 1.0;
  rot(2, 2) = 1.0;
  rot += vx;
  rot += (1.0 / (1.0 + con)) * vx * vx;
  std::cout << rot << "\n is the rotation matrix\n";
  c2.rotate(rot);
  axis1 = c1.GetAxis();
  axis2 = c2.GetAxis();
  std::cout << "Axis 1" << axis1 << "\n";
  std::cout << "Axis 2" << axis2 << "\n";
  // Shifting
  c2.shift(c1.trajectory[c1.trajectory.size() - 1]);
  std::cout << c1.get_width() << ", " << c2.get_width() << "\n";
  for (int i = 0; i < c2.points_roof.size(); i++) {
    answer.points_roof.push_back(c2.points_roof[i]);
  }
  for (int i = 0; i < c2.points_left_wall.size(); i++) {
    answer.points_left_wall.push_back(c2.points_left_wall[i]);
  }
  for (int i = 0; i < c2.points_right_wall.size(); i++) {
    answer.points_right_wall.push_back(c2.points_right_wall[i]);
  }
  for (int i = 0; i < c2.trajectory.size(); i++) {
    answer.trajectory.push_back(c2.trajectory[i]);
  }
  return answer;
}

#endif
