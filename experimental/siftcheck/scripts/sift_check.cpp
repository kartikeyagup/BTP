#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "SiftGPU/SiftGPU.h"
#include <string>
#include <iostream>
#include <vector>

using namespace cv;
// using namespace SiftGPU;
using namespace std;

int main(int argc, char **argv)
{
    SiftGPU sift;
    char * test [] = {"-fo","-1","-v","1"};
    sift.ParseParam(3,test);
    int support  = sift.CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        return 0; 
    sift.RunSIFT("../scripts/img1.jpg");
    // sift.SaveSIFT("img1.sift");
    int num1 = sift.GetFeatureNum();

    std::vector<float> des1(128*num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);

    sift.GetFeatureVector(&keys1[0], &des1[0]);

    sift.RunSIFT("../scripts/img2.jpg");
    // sift.SaveSIFT("img2.sift");
    int num2 = sift.GetFeatureNum();

    std::vector<float> des2(128*num2);
    std::vector<SiftGPU::SiftKeypoint>keys2(num2);
    sift.GetFeatureVector(&keys2[0], &des2[0]);

    SiftMatchGPU matcher(4096);
    matcher.VerifyContextGL();

    matcher.SetDescriptors(0, num1, &des1[0]); //image 1
    matcher.SetDescriptors(1, num2, &des2[0]); //image 2

    int (*match_buf)[2] = new int[num1][2];

    int num_match = matcher.GetSiftMatch(num1, match_buf);

    std::cout << num_match << "   sift matches were found;\n";

    std::vector<SiftGPU::SiftKeypoint> first;
    std::vector<SiftGPU::SiftKeypoint> second;
    
    for(int i = 0 ; i < num_match ; i++)
    {
        SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];//feature point of first image
        SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];//corresponding feature point of second image
        first.push_back(keys1[match_buf[i][0]]);
        second.push_back(keys2[match_buf[i][1]]);
        // cout << key1.x << "   " << key2.x << "   " << key1.y << "   " << key2.y << "\n";
    }
    cv::Mat im1 = cv::imread("../scripts/img1.jpg");
    cv::Mat im2 = cv::imread("../scripts/img2.jpg");

    for(int i = 100 ; i < 120 ; i++){     
        cv::circle(im1, cv::Point(first[i].x,first[i].y), 5, cv::Scalar(255,0,0),1,8,0);
        cv::circle(im2, cv::Point(second[i].x,second[i].y), 5, cv::Scalar(255,0,0),1,8,0);
    }

    // cv::imshow("im1",im1);
    // cv::imshow("im2",im2);

    cv::Size sz1 = im1.size();
    cv::Size sz2 = im2.size();
    cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
    cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
    cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
    im1.copyTo(left);
    im2.copyTo(right);
    
    for(int i = 100 ; i < 120 ; i++){
        cv::line(im3, Point(first[i].x,first[i].y), Point(second[i].x + sz1.width,second[i].y),Scalar(255,0,0),1, 8, 0);
    }

    cv::imshow("im3", im3);
    cv::waitKey(0);

    std::cout << "Completed\n";

    return 0;
}