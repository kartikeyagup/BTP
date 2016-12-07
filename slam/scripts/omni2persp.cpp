#include "undistort.h"
#include "FishCam.h"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>

int main()
{
  FishOcam f;
  std::string s = "wide/calib_results.txt";
  f.init(s);
  CVec2Img warp;
  int hout;
  const int wout = f.width;
  double hfov;
  double vfov;
  double focal;
  f.createPerspectiveWarp(warp, hout, hfov, vfov, focal, 1920, 1080, 1920, true);
  std::cout << "Focal is " << focal << "\n";
  std::cout << "Hout is " << hout << "\n";
  std::string file_or_video_name = "wide/hostel.MP4";
  std::string out_video = "wide/hostel_out.avi";
  cv::VideoCapture inputVideo(file_or_video_name);
  cv::VideoWriter outputVideo;
  int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));
  cv::Size S = cv::Size(wout, hout);
  
  int waitTime=0;

  if (!(inputVideo.isOpened())) {
    // check if we succeeded
    std::cerr << "Invalid video file\n";
    return -1;
  }

  outputVideo.open(out_video, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);


  if (!outputVideo.isOpened())
  {
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
    for (int i = 0; i < warp.rows; i++)
    {
      for (int j = 0; j < warp.cols; j++)
      {
        CVec2f tmp = warp.arr[i][j];
        image2.at<cv::Vec3b>(i, j) = image.at<cv::Vec3b>(tmp.y, tmp.x);
      }
    }
    outputVideo << image2;
  }
}
