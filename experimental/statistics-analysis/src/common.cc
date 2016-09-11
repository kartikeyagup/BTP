#include "common.h"

void UpdatePosition(camera_frame &frame, 
  float distance, 
  motion_type direction) {
  float dx, dz;
  switch (direction) {
  case FORWARD:
    dx = 0;
    dz = -1;
    break;
  case LEFT:
    dx = -1;
    dz = 0;
    break; 
  }
  frame.position.x += dx*distance;
  frame.position.z += dz*distance;
}

float getColorR(int i, int j, grid_params &grid_description) {
  return (1.0*grid_description.p2c[TwoDPoint(i, j)].R)/255.0;
}

float getColorG(int i, int j, grid_params &grid_description) {
  return (1.0*grid_description.p2c[TwoDPoint(i, j)].G)/255.0;
}

float getColorB(int i, int j, grid_params &grid_description) {
  return (1.0*grid_description.p2c[TwoDPoint(i, j)].B)/255.0;
}
