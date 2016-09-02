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
    cv::Point3f starting_point(0, 0, 0);

    simulate_images(grid_description,
        FORWARD, 0, 5, 10,
        intrinsics, starting_point,
        output_frames);

    return 0;
}

