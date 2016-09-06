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

float getColorR(int i, int j) {
  return 0.1 + i*0.18;
}

float getColorG(int i, int j) {
  return 1 - j*0.18;
}

float getColorB(int i, int j) {
  return 0.5 + (i+j)*0.05;
}
