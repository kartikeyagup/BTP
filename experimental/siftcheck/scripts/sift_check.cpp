#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "SiftGPU/SiftGPU.h"
#include <string>
#include <iostream>
#include <vector>
#include <math.h>

using namespace cv;
// using namespace SiftGPU;
using namespace std;

float dot_product(std::vector<float> des1, std::vector<float> des2, int index1, int index2)
{
    float ans = 0.0;
    for(int i = 0 ; i < 128 ; i++)
    {
        ans += (des1[index1*128 + i] - des2[index2*128 + i])*(des1[index1*128 + i] - des2[index2*128 + i]);
    }
    return sqrt(ans);
}

int main(int argc, char **argv)
{
    SiftGPU sift;
    std::string im1_loc = "../../../../siftimages/img1.jpg";
    std::string im2_loc = "../../../../siftimages/img2.jpg";
    // char * test [] = {"-fo","-1","-v","1"};
    // sift.ParseParam(3,test2);
    int support  = sift.CreateContextGL();
    if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        return 0; 
    sift.RunSIFT(im1_loc.c_str());
    // sift.SaveSIFT("img1.sift");
    int num1 = sift.GetFeatureNum();

    std::vector<float> des1(128*num1);
    std::vector<SiftGPU::SiftKeypoint> keys1(num1);

    sift.GetFeatureVector(&keys1[0], &des1[0]);

    sift.RunSIFT(im2_loc.c_str());
    // sift.SaveSIFT("img2.sift");
    int num2 = sift.GetFeatureNum();

    std::vector<float> des2(128*num2);
    std::vector<SiftGPU::SiftKeypoint>keys2(num2);
    sift.GetFeatureVector(&keys2[0], &des2[0]);

    SiftMatchGPU matcher(4096);
    matcher.VerifyContextGL();

    matcher.SetDescriptors(0, num1, &des1[0]); //image 1
    matcher.SetDescriptors(1, num2, &des2[0]); //image 2

    // matcher.SetDescriptors(1, num1, &des1[0]); //image 1
    // matcher.SetDescriptors(0, num2, &des2[0]); //image 2

    int (*match_buf)[2] = new int[num1][2];

    // int (*match_buf)[2] = new int[num2][2];

    int num_match = matcher.GetSiftMatch(num1, match_buf);

    // int num_match = matcher.GetSiftMatch(num2, match_buf);

    std::cout << num_match << "   sift matches were found;\n";

    std::vector<SiftGPU::SiftKeypoint> first;
    std::vector<SiftGPU::SiftKeypoint> second;
     
    SiftGPU::SiftKeypoint key1;
    SiftGPU::SiftKeypoint key2;
    int index1;
    int index2;
    float dot_prod;

    for(int i = 0 ; i < num_match ; i++)
    {
        index1 = match_buf[i][0];
        index2 = match_buf[i][1];
        key1 = keys1[index1];//feature point of first image
        key2 = keys2[index2];//corresponding feature point of second image

        dot_prod = dot_product(des1,des2,index1,index2);

        // cout << "dot product is : " << dot_prod << "\n";

        if(dot_prod < 0.35)
        {
            cout << dot_prod << "\n";
            first.push_back(keys1[match_buf[i][0]]);
            second.push_back(keys2[match_buf[i][1]]);
            cout << key1.x << "   " << key2.x << "   " << key1.y << "   " << key2.y << "\n";
        }
    }

    for(int i = 0 ; i < first.size() ; i++)
    {
        std::cout << first[i].x << "  " << second[i].x << "  " << first[i].y << "  " << second[i].y << "\n";
    }

    cout << "number of matches now is : " << first.size() << "\n";

    cv::Mat im1 = cv::imread(im1_loc);
    cv::Mat im2 = cv::imread(im2_loc);

    cv::Size sz1 = im1.size();
    cv::Size sz2 = im2.size();

    cout << "size of image 1   " << sz1.height << "   " << sz1.width << "\n";
    cout << "size of image 1   " << sz2.height << "   " << sz2.width << "\n";


    for(int i = 0; i < first.size() ; i++)
    {     
        cv::circle(im1, cv::Point(first[i].x,first[i].y), 5, cv::Scalar(255,0,0),1,8,0);
        cv::circle(im2, cv::Point(second[i].x,second[i].y), 5, cv::Scalar(255,0,0),1,8,0);
    }

    cv::imshow("im1",im1);
    cv::imshow("im2",im2);

    
    cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
    cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
    cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
    im1.copyTo(left);
    im2.copyTo(right);
    
    for(int i = 0 ; i < first.size(); i++)
    {
        cv::line(im3, Point(first[i].x,first[i].y), Point(second[i].x + sz1.width,second[i].y),Scalar(255,0,0),1, 8, 0);
    }

    cv::imshow("im3", im3);
    cv::waitKey(0);

    std::cout << "Completed\n";

    return 0;
}