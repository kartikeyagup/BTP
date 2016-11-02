#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <utility>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <stdio.h>
#include <stdlib.h>
#include "tracking_helpers.h"
#include "sift_processing.h"
#include "helpers.h"
#include "gflags/gflags.h"
#include "correspondance.h"

DEFINE_string(dirname, "data2", "Directory to dump in");
DEFINE_string(video, "vid3.MP4", "Name of the video");
DEFINE_int32(keyframe, 30, "Max number of frames in a keyframe");
DEFINE_int32(chunks, 101, "Max number of keyframes in a chunk");
DEFINE_int32(overlap, 10, "Number of frames to be considered in the overalp");
DEFINE_bool(corres, false, "Dump image correspondances");
DEFINE_bool(undistort, false, "Undistort the images");
DEFINE_bool(use_sift, false, "Use sift for corresponances");
DEFINE_int32(min_corners, 5000, "Minimum number of points in image below which more will be added");
DEFINE_int32(loop_closure_size, 50, "Number of key frames over which loop closure is applied");
DEFINE_int32(kf_overlap, 1, "Number of keyframes to be overlapped");

float focal = 1134.0/1280; // gopro3
// float focal = 1170.0/1280; // gopro4
// float focal = 809.0/1280.0; // pointgrey
int cx = 640;
int cy = 360;
constexpr int windows_size = 5;

int main(int argc, char **argv)
{
  google::SetUsageMessage("slam --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
  
  cv::VideoCapture cap(FLAGS_video);
  if (!cap.isOpened())
      return 1;
  cv::namedWindow("new", 0);
  
  int numoutmatches;
  int framid = 0;
  int maxCorners = 10000;
  double qualityLevel = 0.01;
  double minDistance = 2;
  int blockSize = 7;
  bool useHarrisDetector = false;
  double k = 0.04;
  std::vector<frame_pts> all_frame_pts;
  std::unordered_map<int, cv::Mat> images;
  std::unordered_map<int, cv::Mat> kf_images;

  cv::Mat newFrame, mask, rawFrame, lastFrame;
  std::vector<cv::Point2f> corners;
  std::vector<int> useless;
  int siftlatest=0;
  int initindex(0), finalindex(0);
  int kfinitindex(0), kfinalindex(0);
  std::vector<frame_pts> keyFrames;
  int prevKeyFrame = 0;

  VerifyTwoViewMatchesOptions options;
  options.bundle_adjustment = false;
  options.min_num_inlier_matches = 10;
  options.estimate_twoview_info_options.max_sampson_error_pixels = 2.25;
  int ct1=0;

  while (true) {
    cap.read(rawFrame);
    if (rawFrame.empty()) {
      ct1++;
      if (ct1<25)
        continue;
      if (all_frame_pts.rbegin()->frame_id != keyFrames.rbegin()->frame_id) {
        cv::imwrite(FLAGS_dirname + "/img_"+std::to_string(all_frame_pts.rbegin()->frame_id)+".jpg", lastFrame);
        keyFrames.push_back(*all_frame_pts.rbegin());
      }
      break;
    }
    ct1 =0;
    rawFrame.copyTo(lastFrame);
    if (FLAGS_undistort) {
      undistort(rawFrame);
    }
    if (framid>0) {
      std::cout << "\rProcessing frame " << framid << " with points: " 
                << all_frame_pts.rbegin()->features.size() << ", Number of keyframes: "
                << keyFrames.size() << std::flush ;
    }
    cv::cvtColor(rawFrame, newFrame, CV_RGBA2GRAY);
    if (framid == 0) {
      newFrame.copyTo(images[0]);
      newFrame.copyTo(kf_images[0]);
      cx = newFrame.cols/2;
      cy = newFrame.rows/2;
      focal *= 2*cx;
      std::cout << "\nCx is: " << cx << "\n";
      std::cout << "Cy is: " << cy << "\n";
      std::cout << "Focal is: " << focal << "\n";
      cv::goodFeaturesToTrack(newFrame,
        corners,
        maxCorners,
        qualityLevel,
        minDistance,
        mask,
        blockSize,
        useHarrisDetector,
        k);
      siftlatest = 0;
      frame_pts f0(0); 
      for (int i=siftlatest; i<siftlatest+corners.size(); i++) {
        f0.features[i] = img_pt(corners[i], rawFrame.at<cv::Vec3b>(corners[i]));
      }
      all_frame_pts.push_back(f0);
      keyFrames.push_back(f0);
      siftlatest = siftlatest+corners.size();
      cv::imwrite(FLAGS_dirname + "/img_" + std::to_string(framid) + ".jpg", rawFrame);
      framid++;
      continue;
    }
    assert(finalindex == framid-1);
    frame_pts last = Track(all_frame_pts[finalindex], images[finalindex], newFrame, framid, cx, cy);
    for (int i=kfinalindex; i>=kfinitindex; i--) {
      assert(kf_images.find(i) != kf_images.end());
      frame_pts temp_track = Track(keyFrames[i], kf_images[i], newFrame, framid, cx, cy);
      add_more_features(last, temp_track);
    }

    for (int i = finalindex -1; i>=initindex; i--) {
      assert(images.find(i) != images.end());
      frame_pts temp_track = Track(all_frame_pts[i], images[i], newFrame, framid, cx, cy);
      add_more_features(last, temp_track);
    }
    
    if (newKeyFrame(*keyFrames.rbegin(), last, FLAGS_keyframe)) {
      cv::imwrite(FLAGS_dirname + "/img_"+std::to_string(framid)+".jpg", rawFrame);
      keyFrames.push_back(last);
      kfinalindex++;
      newFrame.copyTo(kf_images[kfinalindex]);
      
      if (kfinalindex - kfinitindex >= FLAGS_loop_closure_size) {
        assert(kf_images.find(kfinitindex) != kf_images.end());
        kf_images.erase(kfinitindex);
        kfinitindex++;
      }
    }
    all_frame_pts.push_back(last);

    newFrame.copyTo(images[framid]);
    finalindex = framid;
    // Delete and add the frame to the images
    if (finalindex-initindex >= FLAGS_loop_closure_size) {
      assert(images.find(initindex) != images.end());
      images.erase(initindex);
      initindex++;
    }

    corners = all_frame_pts.rbegin()->get_vector(useless);
    framid++;
    
    if ((framid %5 == 0) || (all_frame_pts.rbegin()->features.size() < FLAGS_min_corners)) {
      mask = cv::Mat::ones(newFrame.size(), CV_8UC1);
      for (int i=0; i<corners.size(); i++) {
        cv::circle(mask, corners[i], 3, cv::Scalar(0), -1);
      }
      corners.clear();
      cv::goodFeaturesToTrack(newFrame,
        corners,
        maxCorners,
        qualityLevel,
        minDistance,
        mask,
        blockSize,
        useHarrisDetector,
        k);
      for (int i=siftlatest; i<siftlatest+corners.size(); i++) {
        assert (all_frame_pts.rbegin()->features.find(i) == all_frame_pts.rbegin()->features.end());
        all_frame_pts.rbegin()->features[i] = img_pt(corners[i-siftlatest], rawFrame.at<cv::Vec3b> (corners[i-siftlatest]));
      }
      siftlatest = siftlatest+corners.size();
      corners = all_frame_pts.rbegin()->get_vector(useless);
    }
    for (int i=0; i<corners.size(); i++) {
      cv::circle(rawFrame, corners[i], 4, cv::Scalar(0), -1);
    }
    cv::imshow("new", rawFrame);
    if (cv::waitKey(100) == 27) 
      break;
  }

  std::cout << "\nStarting Chunk Generation\n";
  std::vector<std::vector<int> > Chunks;
  std::vector<std::vector<int> > Chunks_Intermediate;
  Chunks_Intermediate.push_back(std::vector<int> ());
  int chunksize = FLAGS_chunks;
  int fid = 0;
  int chi = 1;
  for (int i=0; i<keyFrames.size();) {
    if (fid%chunksize==0) {
      Chunks.push_back(std::vector<int> {});
    }

    if ((fid>chi*chunksize - windows_size) and (fid<chi*chunksize + windows_size)) {
      Chunks_Intermediate[Chunks_Intermediate.size()-1].push_back(i);
    } else if (fid == chi*chunksize+ windows_size) {
      chi++;
      Chunks_Intermediate.push_back(std::vector<int> {});
    }
    Chunks[Chunks.size()-1].push_back(i);

    fid++;
    i++;
    if (fid == chunksize ) {
      fid=0;
      i -= FLAGS_kf_overlap;
    }
  }

  std::cout << "Divided " << keyFrames.size() << " keyframes to " << Chunks.size() << " chunks\n";

  std::ofstream listfocalglobal, inifile;
  listfocalglobal.open(FLAGS_dirname + "/list_focal.txt");
  inifile.open(FLAGS_dirname + "/clusternames.ini");

  for (int ch=0; ch<Chunks.size(); ch++) {
    std::vector<int> all_corr_ids = Chunks[ch];
  
    system(("mkdir " + FLAGS_dirname + "/batch_" + std::to_string(ch)).c_str());
    std::ofstream corresfile, rdata, tdata, edata, pdata, listfocal, list_focal, num_cors;
    rdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/R5point.txt");
    tdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/T5point.txt");
    edata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/E5point.txt");
    pdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/original_pairs5point.txt");
    listfocal.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/listsize_focal1.txt");
    list_focal.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/list_focal.txt");
    num_cors.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/num_cors.txt");
    FILE *fp = fopen((FLAGS_dirname + "/batch_" + std::to_string(ch) + "/matches_forRtinlier5point.txt").c_str(), "w");
    fprintf(fp, "                                  \n");

    std::vector<corr> new_compressed;
    for (int i=0; i<all_corr_ids.size(); i++) {
      for (int j=1; j <= FLAGS_overlap && (i+j <all_corr_ids.size()); j++) {
        corr compressed = compress(keyFrames[all_corr_ids[i]], keyFrames[all_corr_ids[i+j]]);
        compressed.frame_1 = i;
        compressed.frame_2 = i+j;
        ChangeCenterSubtracted(compressed, cx, cy);
        new_compressed.push_back(compressed);        
      }
    }
    std::unordered_map<int, int> sift_count;
    std::unordered_map<int, std::unordered_set<int> > sift_frames;
    std::vector<corr> all_corr = new_compressed;
    std::cout << "Done with making a batch\n";
    numoutmatches = 0;
    for (int i=0; i<all_corr.size(); i++) {
      TwoViewInfo twoview_info;
      std::vector<int> inliers;
      if (GetEssentialRT(all_corr[i], twoview_info, inliers, options, focal)) {
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
        for (int j = 0; j<inliers.size(); j++) {
          int loc = inliers[j];
          int sftid = all_corr[i].unique_id[loc];
          int f1 = all_corr[i].frame_1;
          int f2 = all_corr[i].frame_2;
          if (sift_frames[sftid].find(f1) == sift_frames[sftid].end()) {
            sift_frames[sftid].insert(f1);
            sift_count[all_corr[i].unique_id[loc]]++;
          }
          if (sift_frames[sftid].find(f2) == sift_frames[sftid].end()) {
            sift_frames[sftid].insert(f2);
            sift_count[all_corr[i].unique_id[loc]]++;
          }
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
    for (int i=0; i<all_corr_ids.size(); i++) {
      listfocal << "img_" << keyFrames[all_corr_ids[i]].frame_id << ".jpg" << " 0 " << focal << "\n";
      list_focal << "img_" << keyFrames[all_corr_ids[i]].frame_id << ".jpg " << focal << "\n";
    }
    int total_size = 0;
    int count = 0;
    std::vector<int> counts;
    for (auto it: sift_count) {
      counts.push_back(it.second);
      count += it.second;
      total_size++;
    }
    std::nth_element(counts.begin(), counts.begin() + total_size/2, counts.end());
    std::cout << "Mean: " << count/total_size << " Median: " << counts[total_size/2] << "\n";
    num_cors << counts[total_size/2] << "\n";
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
    num_cors.close();
  }

  std::cout << "Number of intermediate chunks : " << Chunks_Intermediate.size() << "\n";
  for (int ch=0; ch<Chunks_Intermediate.size(); ch++) {
    std::ofstream rdata;
    rdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/IntermediateRT.txt");
    std::vector<int> all_corr_ids = Chunks_Intermediate[ch];
    std::vector<corr> new_compressed;
    for (int i=0; i<all_corr_ids.size(); i++) {
      for (int j=1; i+j<all_corr_ids.size(); j++) {
        corr compressed = compress(keyFrames[all_corr_ids[i]], keyFrames[all_corr_ids[i+j]]);
        ChangeCenterSubtracted(compressed, cx, cy);
        new_compressed.push_back(compressed);
      }
    }
    std::vector<corr> all_corr = new_compressed;
    std::cout << "Done with Generating intermediate file\n";
    numoutmatches = 0;
    for (int i=0; i<all_corr.size(); i++) {
      TwoViewInfo twoview_info;
      std::vector<int> inliers;
      if (GetEssentialRT(all_corr[i], twoview_info, inliers, options, focal)) {
        if (not((inliers.size() > 0.8*all_corr[i].p1.size()) && (inliers.size() >100))) {
          std::cout << "Removing pair for " << all_corr[i].frame_1 << "\t" << all_corr[i].frame_2 << "\n";
          continue;
        }
        rdata << all_corr[i].frame_1 << " " << all_corr[i].frame_2 << "\n";
        rdata << twoview_info.rotationmat_2(0,0) << " " << twoview_info.rotationmat_2(0,1) << " " << twoview_info.rotationmat_2(0,2) << " " <<
                 twoview_info.rotationmat_2(1,0) << " " << twoview_info.rotationmat_2(1,1) << " " << twoview_info.rotationmat_2(1,2) << " " <<
                 twoview_info.rotationmat_2(2,0) << " " << twoview_info.rotationmat_2(2,1) << " " << twoview_info.rotationmat_2(2,2) << "\n";
        rdata << twoview_info.translation_2(0) << " " << twoview_info.translation_2(1) << " " << twoview_info.translation_2(2) <<"\n";
        
        rdata << inliers.size() << "\n";
        for (int j = 0; j<inliers.size(); j++) {
          int loc = inliers[j];
          int sftid = all_corr[i].unique_id[loc];
          int f1 = all_corr[i].frame_1;
          int f2 = all_corr[i].frame_2;
          rdata << (int) all_corr[i].col[loc].val[0] << " "
                << (int) all_corr[i].col[loc].val[1] << " "
                << (int) all_corr[i].col[loc].val[2] << " "
                << all_corr[i].unique_id[loc] << " "
                << all_corr[i].p1[loc].x << " " 
                << all_corr[i].p1[loc].y << " " 
                << all_corr[i].unique_id[loc] << " "
                << all_corr[i].p2[loc].x << " " 
                << all_corr[i].p2[loc].y << "\n";
        }
      } else {
        std::cerr << "Something bad still happened in intermediate!!!!!!!!!!\n";
      }
    }
    rdata.close();
  }

  for (auto it: keyFrames){
    listfocalglobal << "img_" << it.frame_id << ".jpg " << focal << "\n"; 
  }
  listfocalglobal.close();
  inifile.close();
  return 0;
}
