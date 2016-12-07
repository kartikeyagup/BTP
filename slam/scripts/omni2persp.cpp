#include "undistort.h"
#include "FishCam.h"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>
#include <gflags/gflags.h>

DEFINE_string(calib, "wide/calib_results.txt", "Path to calibration file");
DEFINE_string(input, "wide/hostel.MP4", "Name of the input distorted video");
DEFINE_string(output, "wide/hostel_out.avi", "Name of the output undistorted video");

int main() {
  std::string s = FLAGS_calib;
  std::string file_or_video_name = FLAGS_input;
  std::string out_video = FLAGS_output;
  FishOcam f;
  f.init(s);
  int hout;
  const int wout = f.width;
  double hfov, vfov, focal;
  f.createPerspectiveWarp(hout, hfov, vfov, focal, 1920, 1080, 1920, true);
  std::cout << "Focal is " << focal << "\n";
  std::cout << "Hout is " << hout << "\n";  
  cv::VideoCapture inputVideo(file_or_video_name);
  cv::VideoWriter outputVideo;
  int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));
  cv::Size S = cv::Size(wout, hout);
  
  int waitTime=0;

  if (!(inputVideo.isOpened())) {
    std::cerr << "Invalid video file\n";
    return -1;
  }

  outputVideo.open(out_video, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);

  if (!outputVideo.isOpened()) {
    std::cout << "Could not open the output video for write: \n";
    return -1;
  }

  int grabber_counter = 0;
  while (1) {
    grabber_counter+=1;
    cv::Mat image;
    inputVideo.read(image);
    if (image.empty()) {
      if (grabber_counter >= 42)
        break;
      continue;
    }
    grabber_counter = 0;

    cv::Mat image2(cv::Size(wout, hout), CV_8UC3);
    f.WarpImage(image, image2);
    outputVideo << image2;
  }
}
