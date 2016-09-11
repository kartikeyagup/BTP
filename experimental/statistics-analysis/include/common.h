#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <math.h>
#include <iostream>
#include <utility>
#include <fstream>
#include <stdlib.h>
#include <unordered_map>
#include <gflags/gflags.h>

#define PI 3.14159265
// using namespace std;

struct TwoDPoint {
  int x;
  int y;

  TwoDPoint() {
    x=0;
    y=0;
  }

  TwoDPoint(int x_input, int y_input) {
    x = x_input;
    y = y_input;
  }

  bool operator==(const TwoDPoint &other) const {
    return ((x==other.x) && (y==other.y));
  }
};

// std::ostream &operator<<(std::ostream &os, TwoDPoint const &p) {
//   return os << "[" << p.x << ", " << p.y << "]";
// }

namespace std {
  template<>
  struct hash<TwoDPoint> {
    std::size_t operator()(const TwoDPoint& k) const {
      using std::size_t;
      return ((std::hash<int>()(k.x)) ^ (std::hash<int>()(k.y)));
    }
  };
}

struct Color {
  int R;
  int G;
  int B;

  Color() {
    R=0;
    G=0;
    B=0;
  }

  Color(int cR,int cG,int cB) {
    R = cR;
    G = cG;
    B = cB;
  };

  bool isblack() {
    return ((R==0) && (B==0) && (G==0));
  }

  bool operator==(const Color &other) const {
    return ((R==other.R) && (G==other.G) && (B==other.B));
  }

};

// std::ostream &operator<<(std::ostream &os, Color const &p) {
//   return os << "[" << p.R << ", " << p.G << ", " << p.B << "]";
// }

namespace std {
  template<>
  struct hash<Color> {
    std::size_t operator()(const Color& k) const {
      using std::size_t;
      return ((std::hash<int>()(k.R)) ^
              (std::hash<int>()(k.G)) ^
               (std::hash<int>()(k.B)));
    }
  };
}


struct grid_params {
  int gridx;
  int gridy;
  std::unordered_map<Color, TwoDPoint> c2p;
  std::unordered_map<TwoDPoint, Color> p2c;

  grid_params() {
    gridx = 1;
    gridy = 1;
    c2p = std::unordered_map<Color, TwoDPoint> ();
    p2c = std::unordered_map<TwoDPoint, Color> ();
  }

  grid_params(int x, int y) {
    gridx = x;
    gridy = y;
    
    int prevColorR = 0;
    int prevColorG = 0;
    int prevColorB = 1;
    for (int i=0; i<gridx; i++) {
      for (int j=0; j<gridy; j++) {
        p2c[TwoDPoint(i,j)] = Color(prevColorR, prevColorG, prevColorB);
        c2p[Color(prevColorR,prevColorG,prevColorB)] = TwoDPoint(i,j);
        prevColorB += 2;
        if (prevColorB >= 256) {
          prevColorB = 1;
          prevColorG += 2;
          if (prevColorG >= 256) {
            prevColorG = 1;
            prevColorR += 2;
          }
        }
      }
    }
  }

  grid_params(const grid_params&obj) {
    gridx = obj.gridx;
    gridy = obj.gridy;
    c2p = obj.c2p;
    p2c = obj.p2c;  
  }
};

enum motion_type {
  FORWARD,
  LEFT
};

struct camera_params {
  float f;
  float cx;
  float cy;

  camera_params() { };

  camera_params(float focal,
    float centerx, float centery) {
    f = focal;
    cx = centerx;
    cy = centery;
  };

  camera_params(const camera_params &obj) {
    f = obj.f;
    cx = obj.cx;
    cy = obj.cy;
  }
};

struct camera_frame {
  cv::Mat image;
  float rotation;
  cv::Point3f position;
  camera_params intrinsics;

  camera_frame() {
  };

  camera_frame(const camera_frame &obj) {
    obj.image.copyTo(image);
    rotation = obj.rotation;
    position = obj.position;
    intrinsics = obj.intrinsics;
  }
};

struct triangulation_bundle {
  camera_frame camera;
  cv::Point2f pt;

  triangulation_bundle() { };

  triangulation_bundle(camera_frame cam, 
    cv::Point2f point) {
    camera = cam;
    pt = point;
  }
};

void UpdatePosition(camera_frame &frame, 
  float distance, motion_type direction);

float getColorR(int i, int j, grid_params &grid_description);
float getColorG(int i, int j, grid_params &grid_description);
float getColorB(int i, int j, grid_params &grid_description);

#endif
