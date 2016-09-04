#include "simulator.h"
#include "statistics.h"
#include "triangulate.h"
#include "common.h"
#include <iostream>

//Main program

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage ./sensitivity <dirname>";
        return 1;
    }

    std::vector<camera_frame> output_frames;
    grid_params grid_description(5, 5);
    camera_params intrinsics(1000, 640, 360);
    cv::Point3f starting_point(0, 0, -500);
    motion_type motion = RIGHT;
    float angle = 0;
    int num_images = 3;
    float distance = 100;

    simulate_images(grid_description,
        motion, angle, num_images, distance,
        intrinsics, starting_point,
        output_frames);    

    std::vector<std::vector<cv::Point3f> > triangulated = 
        detect_triangulate(output_frames);

    for (int i=0; i<5; i++) {
        for (int j=0; j<5; j++) {
            std::cout <<i<<"\t" << j <<"\t" << triangulated[i][j] <<"\n";
        }
    }

    dump_disk(triangulated,
        grid_description,
        motion,
        angle,
        num_images,
        distance,
        intrinsics,
        starting_point,
        argv[1]);

    return 0;
}
