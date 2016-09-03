#include "simulator.h"
#include "statistics.h"
#include "triangulate.h"
#include "common.h"
#include <iostream>

//Main program

int main(int argc, char **argv) {
    std::cerr << "Entered main function\n";

    std::vector<camera_frame> output_frames;
    grid_params grid_description(5, 5);
    camera_params intrinsics(100, 100, 960, 540);
    cv::Point3f starting_point(0, 0, -500);

    simulate_images(grid_description,
        RIGHT, 30, 10, 100,
        intrinsics, starting_point,
        output_frames);    
    cv::namedWindow("Display Images");
    for (int i=0; i< output_frames.size(); i++) {
        cv::imshow("Display Images", output_frames[i].image);
        cv::waitKey(0);
    }
    return 0;
}
