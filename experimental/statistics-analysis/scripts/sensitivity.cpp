#include "simulator.h"
#include "statistics.h"
#include "triangulate.h"
#include "common.h"
#include <iostream>

//Main program
DEFINE_double(distance, 100.0, "Distance  between images");
DEFINE_int32(num_images, 3, "Number of images");
DEFINE_double(starting_x, 0, "Starting x point");
DEFINE_double(starting_y, 0, "Starting y point");
DEFINE_double(starting_z, 1000, "Starting z point");
DEFINE_string(dirname, "tempdir", "Directory to dump in");
DEFINE_double(angle, 45, "Angle at which images are taken");
DEFINE_bool(verbose, false, "Print Statements");
DEFINE_bool(dump_images, false, "Store Images");
DEFINE_bool(straight_motion, false, "Move straight");

int main(int argc, char **argv) {
    gflags::SetUsageMessage("sensitivity --help");
    gflags::SetVersionString("1.0.0");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    
    std::vector<camera_frame> output_frames;
    grid_params grid_description(5, 5);
    camera_params intrinsics(0.57735*360, 640, 360);
    cv::Point3f starting_point(FLAGS_starting_x, 
                               FLAGS_starting_y,
                               FLAGS_starting_z);
    motion_type motion = LEFT;
    if (FLAGS_straight_motion) {
        motion = FORWARD;
    }
    float angle = FLAGS_angle;
    int num_images = FLAGS_num_images;
    float distance = FLAGS_distance;

    simulate_images(grid_description,
        motion, angle, num_images, distance,
        intrinsics, starting_point,
        output_frames);    

    std::vector<std::vector<cv::Point3f> > triangulated = 
        detect_triangulate(output_frames);

    if (FLAGS_verbose) {
        for (int i=0; i<5; i++) {
            for (int j=0; j<5; j++) {
                std::cout <<i<<"\t" << j <<"\t" << triangulated[i][j] <<"\n";
            }
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
        output_frames,
        FLAGS_dirname,
        FLAGS_dump_images);
    gflags::ShutDownCommandLineFlags();
    return 0;
}
