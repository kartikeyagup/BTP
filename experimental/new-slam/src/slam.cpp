#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>

struct corr {
  int frame_1;
  int frame_2;
  std::vector<int> unique_id;
  std::vector<cv::Point2f> p1;
  std::vector<cv::Point2f> p2; 

  corr() {};

  corr(int f1, int f2) {
    frame_1 = f1;
    frame_2 = f2;
  }

  corr(const corr& other) {
    frame_1 = other.frame_1;
    frame_2 = other.frame_2;
    unique_id = other.unique_id;
    p1 = other.p1;
    p2 = other.p2;
  }
};

corr CompressCorr(corr &init, corr &final) {
  assert(init.frame_2 == final.frame_1);
  corr compressed(init.frame_1, final.frame_2);
  int finalcorr=0;
  int limitfinal = final.p1.size();
  for (int i=0; (i<init.p1.size()) && (finalcorr<limitfinal); i++) {
    if (init.p2[i]==final.p1[finalcorr]) {
      compressed.unique_id.push_back(init.unique_id[i]);
      compressed.p1.push_back(init.p1[i]);
      compressed.p2.push_back(final.p2[finalcorr]);
      finalcorr++;
    }
  }
  return compressed;
}

void GetGoodPoints(std::vector<cv::Point2f> &prevtracking,
  std::vector<cv::Point2f> &inversetracking, 
  std::vector<uchar> &status) {
  for (int i=0; i<prevtracking.size(); i++)  {
    status[i]=0;
    cv::Point2f temmpPoint = inversetracking[i]-prevtracking[i];
    float magnitude = (temmpPoint.x)*(temmpPoint.x) + (temmpPoint.y)*(temmpPoint.y);
    if (magnitude<=5.0) {
      status[i]=1;
    }
  }
}

void ChangeCenterSubtracted(corr &p) {
  for (int i=0; i<p.p1.size(); i++) {
    p.p1[i].x -= 320;
    p.p1[i].y -= 240; 
    p.p2[i].x -= 320;
    p.p2[i].y -= 240; 
  }
}

void GetEssentialRT(corr &corres, cv::Mat &essential, cv::Mat &R, cv::Mat &T) {
  float f = 100;
  essential = cv::findEssentialMat(corres.p1, corres.p2, f);
  cv::recoverPose(essential, corres.p1, corres.p2, R, T, f);
}

int main(int argc, char const *argv[])
{
  // Open video stream
  cv::VideoCapture cap("vid1.mp4");
  if (!cap.isOpened())
      return 1;
  cv::namedWindow("new", 0);

  std::ofstream corresfile, rdata, tdata, edata, pdata;
  corresfile.open("data/correspondance.txt");
  rdata.open("data/rotation.txt");
  tdata.open("data/translation.txt");
  edata.open("data/essential.txt");
  pdata.open("data/pairs5point.txt");
  int frameskip=1;
  int framid = 0;
  int prevframe = 0;
  int maxCorners = 1000;
  double qualityLevel = 0.03;
  double minDistance = 2;
  int blockSize = 7;
  bool useHarrisDetector = false;
  double k = 0.04;
  std::vector<corr> all_corr;

  std::vector<cv::Mat> all_images;
  cv::Mat oldFrame, newFrame, mask, rawFrame, tempFrame;
  std::vector<cv::Point2f> corners, corners_prev, corners_inverse;
  std::vector<uchar> status, status_inverse;
  std::vector<float> err;
  cv::Size winSize(15, 15);
  cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.03);
  std::vector<int> siftids;
  int siftlatest=0;
  while (true) {
    std::cerr << framid <<"\n";
    cap >> rawFrame;
    cv::cvtColor(rawFrame, newFrame, CV_RGBA2GRAY);
    if (framid<=30) {
      newFrame.copyTo(oldFrame);
      rawFrame.copyTo(tempFrame);
      all_images.push_back(tempFrame);
      cv::goodFeaturesToTrack(oldFrame,
        corners_prev, 
        maxCorners, 
        qualityLevel, 
        minDistance,
        mask,
        blockSize,
        useHarrisDetector,
        k);
      siftlatest = 0;
      siftids = std::vector<int> ();
      for (int i=siftlatest; i<siftlatest+corners_prev.size(); i++) {
        siftids.push_back(i);
      }
      siftlatest = siftlatest+corners_prev.size();
      prevframe = framid;
      // cv::imwrite("data/img_" + std::to_string(framid) + ".jpg", prevframe);
      framid++;
      continue;
    }

    // KL Tracker
    if (corners_prev.size()>0) {
      calcOpticalFlowPyrLK(oldFrame, newFrame,
        corners_prev, corners, status,
        err, winSize, 2, termcrit, 0, 0.001);

      calcOpticalFlowPyrLK(newFrame, oldFrame,
        corners, corners_inverse, status_inverse,
        err, winSize, 2, termcrit, 0, 0.001);
      
      GetGoodPoints(corners_prev,corners_inverse,status);

      // Store Good Tracks;
      corr frame_corr(prevframe, framid);
      for (int i=0; i<corners.size(); i++) {
        if (status[i]) {
          frame_corr.p1.push_back(corners_prev[i]);
          frame_corr.p2.push_back(corners[i]);
          frame_corr.unique_id.push_back(siftids[i]);
        }
      }
      all_corr.push_back(frame_corr);
      cv::Mat newImg;
      rawFrame.copyTo(newImg);
      all_images.push_back(newImg);

      newFrame.copyTo(oldFrame);
      std::vector<int> newsiftids;
      corners_prev.clear();
      for (int i=0; i<corners.size(); i++) {
        if (status[i]) {
          corners_prev.push_back(corners[i]);
          newsiftids.push_back(siftids[i]);
        }
      }
      siftids = newsiftids;
      assert(siftids.size() == corners_prev.size());
    }

    // cv::imwrite("data/img_"+std::to_string(framid)+".jpg", newFrame);
    cv::imshow("new", newFrame);
    if (cv::waitKey(10) == 27) 
      break;
    prevframe = framid;
    framid++;

    if (framid %5 == 0) {
      mask = cv::Mat::ones(oldFrame.size(), CV_8UC1);
      for (int i=0; i<corners_prev.size(); i++) {
        cv::circle(mask, corners_prev[i], 3, cv::Scalar(0), -1);
      }
      std::vector<cv::Point2f> newcorners;
      cv::goodFeaturesToTrack(oldFrame,
        newcorners, 
        maxCorners, 
        qualityLevel, 
        minDistance,
        mask,
        blockSize,
        useHarrisDetector,
        k);
      corners_prev.insert(corners_prev.end(),
        newcorners.begin(),
        newcorners.end());
      for (int i=siftlatest; i<siftlatest+newcorners.size(); i++) {
        siftids.push_back(i);
      }
      siftlatest = siftlatest+newcorners.size();
    }
  }
  // Correspondance compression.
  int corres_skip=5;
  std::vector<corr> compressed_all;
  for (int i=0; i<all_corr.size();) {
    corr compressed = all_corr[i];
    for (int j=1; j<corres_skip && (i+j<all_corr.size()); j++) {
      compressed = CompressCorr(compressed, all_corr[i+j]);
    }
    std::cout << compressed.frame_1 << "\t" << compressed.frame_2 << "\n";
    cv::imwrite("data/img_" + std::to_string(i/corres_skip) + ".jpg", all_images[compressed.frame_1]);
    cv::imwrite("data/img_" + std::to_string(1 + (i/corres_skip)) + ".jpg", all_images[compressed.frame_2]);
    std::cout << compressed.frame_1 << "\t" << compressed.frame_2 << "\n";
    compressed.frame_1 = i/corres_skip;
    compressed.frame_2 = 1+compressed.frame_1;
        
    ChangeCenterSubtracted(compressed);
    compressed_all.push_back(compressed);
    i=i+corres_skip;
  }

  all_corr = compressed_all;
  
  std::vector<corr> new_compressed;
  for (int i=0; i<all_corr.size(); i++) {
    corr compressed = all_corr[i];
    new_compressed.push_back(compressed);
    for (int j=1; j<5 && (i+j < all_corr.size()); j++) {
      compressed = CompressCorr(compressed, all_corr[i+j]);
      new_compressed.push_back(compressed);
    }
  }
  all_corr = new_compressed;

  for (auto it : all_corr) {
    corresfile << it.frame_1 << "\t" << it.frame_2 << "\t:\t";
    for (int i=0; i< it.p1.size(); i++) {
      corresfile << it.unique_id[i] << "\t" << it.p1[i] << "\t:\t" << it.p2[i] << ";\t";
    }
    corresfile << "\n";
  }
  for (int i=0; i<all_corr.size(); i++) {
    pdata << all_corr[i].frame_1 << "\t" << all_corr[i].frame_2 << "\t1.00000\t1.00000\n";
    cv::Mat R,T,E;
    GetEssentialRT(all_corr[i], E, R, T);
    edata << E.at<double>(0,0) << "\t" << E.at<double>(0,1) << "\t" << E.at<double>(0,2) << "\t" <<
    E.at<double>(1,0) << "\t" << E.at<double>(1,1) << "\t" << E.at<double>(1,2) << "\t" <<
    E.at<double>(2,0) << "\t" << E.at<double>(2,1) << "\t" << E.at<double>(2,2) << "\n";
    // edata << E << "\n";
    tdata << T.at<double>(0) << "\t" << T.at<double>(1) << "\t" << T.at<double>(2) <<"\n";
    // tdata << T << "\n";
    rdata << R.at<double>(0,0) << "\t" << R.at<double>(0,1) << "\t" << R.at<double>(0,2) << "\t" <<
    R.at<double>(1,0) << "\t" << R.at<double>(1,1) << "\t" << R.at<double>(1,2) << "\t" <<
    R.at<double>(2,0) << "\t" << R.at<double>(2,1) << "\t" << R.at<double>(2,2) << "\n";
    // rdata << R << "\n";
  }

  corresfile.close();
  rdata.close();
  tdata.close();
  pdata.close();
  edata.close();
  // essential matrix and decomposition
  return 0;
}