#include <math.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

Eigen::Vector2f project(float focal, int cx, int cy, Eigen::Vector3f pt,
                        Eigen::Vector3f cam) {
  Eigen::Vector2f answer;
  Eigen::Vector3f temp = pt - cam;
  answer(0, 0) = -((focal * temp(0, 0)) / temp(2, 0));
  answer(1, 0) = -((focal * temp(1, 0)) / temp(2, 0));
  return answer;
}

int main(int argc, char **argv) {
  float f = 0.57735 * 360;
  // float f = 369.504;
  int cx = 640;
  int cy = 360;
  int intf = lrint(f);

  std::cout << "int " << intf << "\n";

  Eigen::Vector3f st(0, 0, 400);
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector2f> > all_rectangles;

  all_rectangles.push_back(
      std::make_pair(Eigen::Vector3f(0, 0, -100), Eigen::Vector2f(256, 256)));
  all_rectangles.push_back(
      std::make_pair(Eigen::Vector3f(700, 500, 0), Eigen::Vector2f(128, 128)));
  all_rectangles.push_back(
      std::make_pair(Eigen::Vector3f(0, 500, -50), Eigen::Vector2f(256, 256)));
  all_rectangles.push_back(std::make_pair(Eigen::Vector3f(-400, 50, 100),
                                          Eigen::Vector2f(128, 128)));
  all_rectangles.push_back(std::make_pair(Eigen::Vector3f(500, -100, -200),
                                          Eigen::Vector2f(512, 512)));
  all_rectangles.push_back(std::make_pair(Eigen::Vector3f(-200, -200, 200),
                                          Eigen::Vector2f(128, 128)));
  all_rectangles.push_back(std::make_pair(Eigen::Vector3f(-600, 200, 50),
                                          Eigen::Vector2f(256, 256)));

  std::vector<std::vector<Eigen::Vector3f> > all_depths(
      2 * cy, std::vector<Eigen::Vector3f>(
                  2 * cx, Eigen::Vector3f(99999, 99999, -99999)));

  for (int ar = 0; ar < all_rectangles.size(); ar++) {
    Eigen::Vector3f cornerleftbottom(all_rectangles[ar].first);
    cornerleftbottom -= Eigen::Vector3f(all_rectangles[ar].second(0, 0) / 2,
                                        all_rectangles[ar].second(1, 0) / 2,
                                        -all_rectangles[ar].second(0, 0) / 2);
    for (int i = 0; i < all_rectangles[ar].second(0, 0); i++) {
      for (int j = 0; j < all_rectangles[ar].second(1, 0); j++) {
        Eigen::Vector3f pt = cornerleftbottom + Eigen::Vector3f(i, j, 0);
        Eigen::Vector2f result = project(f, cx, cy, pt, st);
        int x = cx + lrint(result(0, 0));
        int y = cy - lrint(result(1, 0));
        // std::cout << pt << "\t" << x << ", " << y << "\n";
        if (x >= 0 and x < 2 * cx and y >= 0 and y < 2 * cy) {
          if (all_depths[y][x](2, 0) < all_rectangles[ar].first(2, 0)) {
            all_depths[y][x] = pt;
          }
        }
      }
    }
  }

  std::ofstream file;
  file.open("depths.txt");
  for (int i = 0; i < 2 * cx; i++) {
    for (int j = 0; j < 2 * cy; j++) {
      file << i << " " << j << " " << all_depths[j][i](0, 0) << " "
           << all_depths[j][i](1, 0) << " " << all_depths[j][i](2, 0) << "\n";
    }
  }
  file.close();

  return 0;
}
