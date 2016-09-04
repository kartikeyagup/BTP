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
    camera_params intrinsics(1000, 640, 360);
    cv::Point3f starting_point(0, 0, -500);

    simulate_images(grid_description,
        RIGHT, 0, 10, 100,
        intrinsics, starting_point,
        output_frames);    

    std::vector<std::vector<cv::Point3f> > triangulated = 
        detect_triangulate(output_frames);

    for (int i=0; i<5; i++) {
        for (int j=0; j<5; j++) {
            std::cout <<i<<"\t" << j <<"\t" << triangulated[i][j] <<"\n";
        }
    }

    return 0;
}
