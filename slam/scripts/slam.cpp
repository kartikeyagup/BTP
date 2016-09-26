#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include <unordered_map>
#include <stdio.h>
#include "correspondance.h"
#include "verify_two_view_matches.h"
#include "feature_correspondence.h"
#include "camera_intrinsics_prior.h"
#include "estimate_twoview_info.h"
#include "twoview_info.h"

DEFINE_string(dirname, "data2", "Directory to dump in");
DEFINE_string(video, "vid3.MP4", "Name of the video");


float focal = 1690;
int cx = 640;
int cy = 360;


bool GetEssentialRT(corr &corres, 
  TwoViewInfo &twoview_info, 
  std::vector<int> &inlier_indices, 
  VerifyTwoViewMatchesOptions& options) {
  CameraIntrinsicsPrior intrinsics1, intrinsics2;
  intrinsics1.focal_length.value = focal;
  intrinsics1.focal_length.is_set = true;
  intrinsics1.principal_point[0].is_set = true;
  intrinsics1.principal_point[0].value = 0.0;
  intrinsics1.principal_point[1].is_set = true;
  intrinsics1.principal_point[1].value = 0.0;
  intrinsics1.aspect_ratio.is_set = true;
  intrinsics1.aspect_ratio.value = 1.0;
  intrinsics1.skew.is_set = true;
  intrinsics1.skew.value = 0.0;
  
  intrinsics2.focal_length.value = focal;
  intrinsics2.focal_length.is_set = true;
  intrinsics2.principal_point[0].is_set = true;
  intrinsics2.principal_point[0].value = 0.0;
  intrinsics2.principal_point[1].is_set = true;
  intrinsics2.principal_point[1].value = 0.0;
  intrinsics2.aspect_ratio.is_set = true;
  intrinsics2.aspect_ratio.value = 1.0;
  intrinsics2.skew.is_set = true;
  intrinsics2.skew.value = 0.0;

  std::vector<FeatureCorrespondence> correspondences;
  for (int i=0; i<corres.p1.size(); i++) {
    FeatureCorrespondence tmp;
    tmp.feature1.x() = corres.p1[i].x;
    tmp.feature1.y() = corres.p1[i].y;
    tmp.feature2.x() = corres.p2[i].x;
    tmp.feature2.y() = corres.p2[i].y;
    correspondences.push_back(tmp);
  }
  return VerifyTwoViewMatches(options, 
      intrinsics1,
      intrinsics2, 
      correspondences, 
      &twoview_info, 
      &inlier_indices);
}

bool VerifyTwoViewMatches(
  const VerifyTwoViewMatchesOptions& options,
  const CameraIntrinsicsPrior& intrinsics1,
  const CameraIntrinsicsPrior& intrinsics2,
  const std::vector<FeatureCorrespondence>& correspondences,
  TwoViewInfo* twoview_info,
  std::vector<int>* inlier_indices) {
  if (correspondences.size() < options.min_num_inlier_matches) {
    return false;
  }

  // Estimate the two view info. If we fail to estimate a two view info then do
  // not add this view pair to the verified matches.
  if (!EstimateTwoViewInfo(options.estimate_twoview_info_options,
    intrinsics1,
    intrinsics2,
    correspondences,
    twoview_info,
    inlier_indices)) {
    return false;
  }

  // If there were not enough inliers, return false and do not bother to
  // (potentially) run bundle adjustment.
  if (inlier_indices->size() < options.min_num_inlier_matches) {
    return false;
  }
  return true;
}

int main(int argc, char **argv)
{
  // Open video stream
  // cv::VideoCapture cap(0);

  gflags::SetUsageMessage("slam --help");
  gflags::SetVersionString("1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  cv::VideoCapture cap(FLAGS_video);
  if (!cap.isOpened())
      return 1;
  cv::namedWindow("new", 0);

  std::ofstream corresfile, rdata, tdata, edata, pdata, listfocal, list_focal;
  rdata.open(FLAGS_dirname + "/R5point.txt");
  tdata.open(FLAGS_dirname + "/T5point.txt");
  edata.open(FLAGS_dirname + "/E5point.txt");
  pdata.open(FLAGS_dirname + "/original_pairs5point.txt");
  listfocal.open(FLAGS_dirname + "/listsize_focal1.txt");
  list_focal.open(FLAGS_dirname + "/list_focal.txt");
  FILE *fp = fopen((FLAGS_dirname + "/matches_forRtinlier5point.txt").c_str(), "w");
  fprintf(fp, "                                  \n");
  
  int numoutmatches = 0; 
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
  std::vector<int> fileids;
  std::unordered_map<int, bool> all_files;
  int siftlatest=0;
  
  VerifyTwoViewMatchesOptions options;
  options.bundle_adjustment = false;
  options.min_num_inlier_matches = 10;
  options.estimate_twoview_info_options.max_sampson_error_pixels = 2.25;


  while (true) {
    cap.read(rawFrame);
    if (rawFrame.empty()) {
      break;
    }
    std::cerr << framid <<"\n";
    cv::cvtColor(rawFrame, newFrame, CV_RGBA2GRAY);
    if (framid == 0) {
      newFrame.copyTo(oldFrame);
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
      cv::imwrite(FLAGS_dirname + "/img_" + std::to_string(framid) + ".jpg", rawFrame);
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
      
      GetGoodPoints(corners_prev,corners_inverse,status,status_inverse);

      // Store Good Tracks;
      corr frame_corr(prevframe, framid);
      for (int i=0; i<corners.size(); i++) {
        if (status[i]) {
          if ((corners[i].x > (2*cx -1)) || (corners[i].y > (2*cy -1)) || (corners[i].x <0) || (corners[i].y<0))
          {
            status[i] = 0;
            continue;
          }
          assert(corners[i].x < 2*cx);
          assert(corners[i].y < 2*cy);
          assert(corners[i].x >= 0);
          assert(corners[i].y >= 0);
          frame_corr.p1.push_back(corners_prev[i]);
          frame_corr.p2.push_back(corners[i]);
          frame_corr.unique_id.push_back(siftids[i]);
        }
      }
      CalculateDelta(frame_corr);
      all_corr.push_back(frame_corr);
      
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

    cv::imwrite(FLAGS_dirname + "/img_"+std::to_string(framid)+".jpg", rawFrame);
    for (int i=0; i<corners_prev.size(); i++) {
      cv::circle(rawFrame, corners_prev[i], 4, cv::Scalar(0), -1);
    }
    cv::imshow("new", rawFrame);
    if (cv::waitKey(1) == 27) 
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
  std::cout << "Starting 1st round of compression\n";
  // Correspondance compression.
  int corres_skip=30;
  std::vector<corr> compressed_all;
  int fid = 0;
  for (int i=0; i<all_corr.size();) {
    corr compressed = all_corr[i];
    std::cerr << "Init delta at " << compressed.frame_1 << "\t" << compressed.delta << "\n";
    for (int j=1; j<corres_skip && (i+j<all_corr.size()) && WithinRange(compressed); j++) {
      compressed = CompressCorr(compressed, all_corr[i+j]);
    }
    std::cerr << "Final delta at " << compressed.frame_2 << "\t" << compressed.delta << "\n";
    i=compressed.frame_2;
    if (all_files.find(compressed.frame_1)==all_files.end())
      fileids.push_back(compressed.frame_1);
    if (all_files.find(compressed.frame_2)==all_files.end())
      fileids.push_back(compressed.frame_2);
    all_files[compressed.frame_1] = true;
    all_files[compressed.frame_2] = true;
    compressed.frame_1 = fid;
    compressed.frame_2 = fid+1;
    fid++;        
    ChangeCenterSubtracted(compressed, cx, cy);
    compressed_all.push_back(compressed);
  }
  std::cout << "Done with 1st round of compression\n";
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
  std::cout << "Done with 2nd round of compression\n";

  for (int i=0; i<all_corr.size(); i++) {
    std::cerr << "Processing " << i << " out of " << all_corr.size() << "\n";
    TwoViewInfo twoview_info;
    std::vector<int> inliers;
    if (GetEssentialRT(all_corr[i], twoview_info, inliers, options)) {
      numoutmatches++;
      rdata << twoview_info.rotationmat_2(0,0) << " " << twoview_info.rotationmat_2(0,1) << " " << twoview_info.rotationmat_2(0,2) << " " <<
               twoview_info.rotationmat_2(1,0) << " " << twoview_info.rotationmat_2(1,1) << " " << twoview_info.rotationmat_2(1,2) << " " <<
               twoview_info.rotationmat_2(2,0) << " " << twoview_info.rotationmat_2(2,1) << " " << twoview_info.rotationmat_2(2,2) << "\n";
      tdata << twoview_info.translation_2(0) << " " << twoview_info.translation_2(1) << " " << twoview_info.translation_2(2) <<"\n";
      edata << twoview_info.essential_mat(0,0) << " " << twoview_info.essential_mat(0,1) << " " << twoview_info.essential_mat(0,2) << " " <<
               twoview_info.essential_mat(1,0) << " " << twoview_info.essential_mat(1,1) << " " << twoview_info.essential_mat(1,2) << " " <<
               twoview_info.essential_mat(2,0) << " " << twoview_info.essential_mat(2,1) << " " << twoview_info.essential_mat(2,2) << "\n";
      pdata << all_corr[i].frame_1 +1 << " " << all_corr[i].frame_2 +1 << " 1.00000 1.00000\n";
      fprintf(fp, "%d %d %ld\n", all_corr[i].frame_1, all_corr[i].frame_2, inliers.size());
      for (int j = 0; j<inliers.size(); j++)
      {
        int loc = inliers[j];
        fprintf(fp, "%d %f %f %d %f %f\n", 
            all_corr[i].unique_id[loc],
            all_corr[i].p1[loc].x, 
            all_corr[i].p1[loc].y, 
            all_corr[i].unique_id[loc],
            all_corr[i].p2[loc].x, 
            all_corr[i].p2[loc].y);
      }
    } else {
      std::cerr << "Something bad still happened!!!!!!!!!!\n";
    }
  }
  for (int i=0; i<fileids.size(); i++) {
    listfocal << "img_" << fileids[i] << ".jpg" << " 0 " << focal << "\n";
    list_focal << "img_" << fileids[i] << ".jpg " << focal << "\n";
  }
  fseek(fp, 0, SEEK_SET);
  fprintf(fp, "%d", numoutmatches);
  fclose(fp);
  corresfile.close();
  rdata.close();
  tdata.close();
  pdata.close();
  edata.close();
  listfocal.close();
  list_focal.close();
  return 0;
}
