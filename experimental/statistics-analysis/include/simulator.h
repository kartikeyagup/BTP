#ifndef HELPERS_H
#define HELPERS_H

#include <GL/freeglut.h>
#include <GL/gl.h>    // Header File For The OpenGL32 Library
#include <GL/glu.h>   // Header File For The GLu32 Library
#include <GL/glut.h>  // Header File For The GLUT Library
#include <stdio.h>    // Header file for standard file i/o.
#include <stdlib.h>   // Header file for malloc/free.
#include <unistd.h>
#include <iostream>
#include "common.h"

void getImage(cv::Mat &image);

void simulate_images(grid_params grid_description, motion_type motion,
                     float angle, int num_images, float distance,
                     camera_params intrinsics, cv::Point3f starting_point,
                     std::vector<camera_frame> &output_frames);

#endif
