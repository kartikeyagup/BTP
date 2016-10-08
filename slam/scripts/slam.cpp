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
#include <stdlib.h>
#include "correspondance.h"
#include "verify_two_view_matches.h"
#include "feature_correspondence.h"
#include "camera_intrinsics_prior.h"
#include "estimate_twoview_info.h"
#include "twoview_info.h"

DEFINE_string(dirname, "data2", "Directory to dump in");
DEFINE_string(video, "vid3.MP4", "Name of the video");
DEFINE_int32(keyframe, 30, "Max number of frames in a keyframe");
DEFINE_int32(chunks, 100, "Max number of keyframes in a chunk");
DEFINE_int32(overlap, 10, "Number of frames to be considered in the overalp");
DEFINE_bool(corres, false, "Dump image correspondances");
DEFINE_bool(undistort, false, "Undistort the images");

float focal = 1134;
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

void ShowCorres(std::string FLAGS_dirname, corr compressed) {
  cv::Mat im1 = cv::imread(FLAGS_dirname+"/img_"+std::to_string(compressed.frame_1)+".jpg");
  cv::Mat im2 = cv::imread(FLAGS_dirname+"/img_"+std::to_string(compressed.frame_2)+".jpg");
  cv::Size sz1 = im1.size();
  cv::resize(im1, im1, cv::Size(sz1.width/2, sz1.height/2));
  cv::resize(im2, im2, cv::Size(sz1.width/2, sz1.height/2));
  sz1 = im1.size();
  system(("mkdir " + FLAGS_dirname + "/corr_" + std::to_string(compressed.frame_1)+"_"+std::to_string(compressed.frame_2)).c_str());
  for (int i=0; i<compressed.p1.size()/10;i++) {
    cv::Mat im3(sz1.height, 2*sz1.width, CV_8UC3);
    cv::Mat left(im3, cv::Rect(0,0,sz1.width, sz1.height));
    im1.copyTo(left);
    cv::Mat right(im3, cv::Rect(sz1.width,0,sz1.width, sz1.height));
    im2.copyTo(right);
    for (int j=10*i; j<10*(i+1); j++) {
      cv::circle(im3, cv::Point2f(compressed.p1[j].x/2,compressed.p1[j].y/2),3, cv::Scalar(255,0,0), -1);
      cv::circle(im3, cv::Point2f(compressed.p2[j].x/2+sz1.width,compressed.p2[j].y/2),3, cv::Scalar(255,0,0), -1);
      cv::line(im3, cv::Point2f(compressed.p1[j].x/2,compressed.p1[j].y/2), cv::Point2f(compressed.p2[j].x/2+sz1.width,compressed.p2[j].y/2), cv::Scalar(0,0,255));
    }
    cv::imwrite(FLAGS_dirname+"/corr_"+std::to_string(compressed.frame_1)+"_"+std::to_string(compressed.frame_2)+"/"+std::to_string(i)+".jpg", im3);
  }
}

void undistort(cv::Mat &img) {
  cv::Mat temp = img.clone();
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
  cameraMatrix.at<double>(0, 0) = 1134;
  cameraMatrix.at<double>(0, 1) = 0;
  cameraMatrix.at<double>(0, 2) = 645;
  cameraMatrix.at<double>(1, 0) = 0;
  cameraMatrix.at<double>(1, 1) = 1126;
  cameraMatrix.at<double>(1, 2) = 364;
  cameraMatrix.at<double>(2, 0) = 0;
  cameraMatrix.at<double>(2, 1) = 0;
  cameraMatrix.at<double>(2, 2) = 1;
  std::vector<float> distCoeffs = {-0.29344778, 0.17523322, 0.00032134, -0.00088967, -0.08528005};
  cv::undistort(temp, img, cameraMatrix, distCoeffs);    
}

int main(int argc, char **argv)
{
  // Open video stream
  // cv::VideoCapture cap(0);

  google::SetUsageMessage("slam --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  cv::VideoCapture cap(FLAGS_video);
  if (!cap.isOpened())
      return 1;
  cv::namedWindow("new", 0);
  
  int numoutmatches; 
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
  std::vector<std::vector<int> > fileids;
  std::vector<std::unordered_map<int, bool> > all_files;
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
    if (FLAGS_undistort) {
      undistort(rawFrame);
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
  int corres_skip= FLAGS_keyframe;
  std::vector<std::vector<corr> > Chunks;
  int chunksize = FLAGS_chunks;
  std::vector<corr> compressed_all;
  int fid = 0;
  for (int i=0; i<all_corr.size();) {
    if (fid%chunksize==0) {
      Chunks.push_back(std::vector<corr> ());
      all_files.push_back(std::unordered_map<int, bool> ());
      fileids.push_back(std::vector<int> ());
    }
    corr compressed = all_corr[i];
    std::cerr << "Init delta at " << compressed.frame_1 << "\t" << compressed.delta << "\n";
    for (int j=1; j<corres_skip && (i+j<all_corr.size()) && WithinRange(compressed); j++) {
      compressed = CompressCorr(compressed, all_corr[i+j]);
    }
    std::cerr << "Final delta at " << compressed.frame_2 << "\t" << compressed.delta << "\n";
    i=compressed.frame_2;

    if (FLAGS_corres) {
      ShowCorres(FLAGS_dirname, compressed);
    }
    
    if (all_files[Chunks.size()-1].find(compressed.frame_1)==all_files[Chunks.size()-1].end())
      fileids[Chunks.size()-1].push_back(compressed.frame_1);
    if (all_files[Chunks.size()-1].find(compressed.frame_2)==all_files[Chunks.size()-1].end())
      fileids[Chunks.size()-1].push_back(compressed.frame_2);
    all_files[Chunks.size()-1][compressed.frame_1] = true;
    all_files[Chunks.size()-1][compressed.frame_2] = true;
    compressed.frame_1 = fid;
    compressed.frame_2 = fid+1;
    fid++;        
    ChangeCenterSubtracted(compressed, cx, cy);
    Chunks[Chunks.size()-1].push_back(compressed);
  }
  std::cout << "Done with 1st round of compression\n";
  std::ofstream listfocalglobal, inifile;
  listfocalglobal.open(FLAGS_dirname + "/list_focal.txt");
  inifile.open(FLAGS_dirname + "/clusternames.ini");

  for (int ch=0; ch<Chunks.size(); ch++) {
  
    all_corr = Chunks[ch];
  
    system(("mkdir " + FLAGS_dirname + "/batch_" + std::to_string(ch)).c_str());
    std::ofstream corresfile, rdata, tdata, edata, pdata, listfocal, list_focal;
    rdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/R5point.txt");
    tdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/T5point.txt");
    edata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/E5point.txt");
    pdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/original_pairs5point.txt");
    listfocal.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/listsize_focal1.txt");
    list_focal.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/list_focal.txt");
    FILE *fp = fopen((FLAGS_dirname + "/batch_" + std::to_string(ch) + "/matches_forRtinlier5point.txt").c_str(), "w");
    fprintf(fp, "                                  \n");

    std::vector<corr> new_compressed;
    for (int i=0; i<all_corr.size(); i++) {
      corr compressed = all_corr[i];
      new_compressed.push_back(compressed);
      for (int j=1; j<FLAGS_overlap && (i+j < all_corr.size()); j++) {
        compressed = CompressCorr(compressed, all_corr[i+j]);
        new_compressed.push_back(compressed);
      }
    }
    all_corr = new_compressed;
    std::cout << "Done with 2nd round of compression\n";
    numoutmatches = 0;
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
        pdata << all_corr[i].frame_1 +1 - chunksize*ch << " " << all_corr[i].frame_2 +1 - chunksize*ch << " 1.00000 1.00000\n";
        fprintf(fp, "%d %d %ld\n", all_corr[i].frame_1 - chunksize*ch , all_corr[i].frame_2 - chunksize*ch , inliers.size());
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
    for (int i=0; i<fileids[ch].size(); i++) {
      listfocal << "img_" << fileids[ch][i] << ".jpg" << " 0 " << focal << "\n";
      list_focal << "img_" << fileids[ch][i] << ".jpg " << focal << "\n";
    }
    for (int i=0; i<fileids[ch].size() -1; i++) {
      listfocalglobal << "img_" << fileids[ch][i] << ".jpg " << focal << "\n";
    }
    inifile << "batch_" << ch << "\n";
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
  }
  listfocalglobal << "img_" << *((*fileids.rbegin()).rbegin()) << ".jpg " << focal << "\n";
  listfocalglobal.close();
  inifile.close();
  return 0;
}
