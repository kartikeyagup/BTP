#include <gflags/gflags.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "FishCam.h"
#include "correspondance.h"
#include "helpers.h"
#include "sift_processing.h"

DEFINE_string(dirname, "data2", "Directory to dump in");
DEFINE_string(video, "vid3.MP4", "Name of the video");
DEFINE_string(calib, "wide/calib_results.txt", "Calibration file");
DEFINE_int32(keyframe, 10, "Max number of frames in a keyframe");
DEFINE_int32(chunks, 150, "Max number of keyframes in a chunk");
DEFINE_int32(overlap, 30, "Number of frames to be considered in the overalp");
DEFINE_int32(
    min_corners, 12000,
    "Minimum number of points in image below which more will be added");
DEFINE_bool(corres, false, "Dump image correspondances");
DEFINE_bool(undistort, false, "Undistort the images");
DEFINE_bool(use_sift, false, "Use sift for corresponances");
DEFINE_bool(clear_non_kf, true, "Remove non kf images");

// float focal = 424.153/1280;
// float focal = 516.5/640;
// float focal = 1701.0/1920;
// float focal = 707.09/1226; // KITTI
// float focal = 538.918/640.0; // TUM fr3
// float focal = 518.1/640.0; // TUM fr1
// float focal = 522.2/640.0; // TUM fr2
// float focal = 428.582/1280; // climbing hyper
// float focal = 1237.14/1101;
// float focal = 1153.12/960; // camvid
float focal = 207.846 / 1280;  // simulated
int cx = 640;
int cy = 360;
constexpr int windows_size = 5;
constexpr int maxCorners = 100000;
constexpr double qualityLevel = 0.001;
constexpr double minDistance = 2;
constexpr int blockSize = 7;
constexpr bool useHarrisDetector = false;
constexpr double k = 0.04;
cv::Size winSize(15, 15);
cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.03);

int main(int argc, char **argv) {
  google::SetUsageMessage("slam --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  cv::VideoCapture cap(FLAGS_video);
  if (!cap.isOpened()) return 1;
  cv::namedWindow("new", 0);
  cv::namedWindow("mask", 1);

  int numoutmatches;
  int framid = 0;
  int prevframe = 0;
  std::vector<corr> all_corr;

  std::vector<cv::Mat> all_images;
  cv::Mat oldFrame, newFrame, mask, rawFrame, tempFrame;
  std::vector<cv::Point2f> corners, corners_prev, corners_inverse;
  std::vector<cv::Vec3b> colors;
  std::vector<uchar> status, status_inverse;
  std::vector<float> err;
  std::vector<int> siftids;
  std::vector<std::vector<int> > fileids;
  std::vector<std::unordered_map<int, bool> > all_files;
  int siftlatest = 0;
  std::unordered_map<int, bool> required_images;
  FishOcam cam_model;
  if (FLAGS_undistort) {
    cam_model.init(FLAGS_calib);
    focal = cam_model.focal;
    std::cout << "Focal " << focal << "\n";
  }

  VerifyTwoViewMatchesOptions options;
  options.bundle_adjustment = false;
  options.min_num_inlier_matches = 10;
  options.estimate_twoview_info_options.max_sampson_error_pixels = 2.25;

  int rc(0);
  while (true) {
    cap.read(rawFrame);
    if (rawFrame.empty()) {
      if (rc > 200) break;
      rc++;
      continue;
    }
    rc = 0;
    if (FLAGS_undistort) {
      cv::Mat undistorted;
      cam_model.WarpImage(rawFrame, undistorted);
      undistorted.copyTo(rawFrame);
    }
    required_images[framid] = true;
    std::cout << "\rOn frame id: " << framid << " and tracking "
              << corners.size() << " points" << std::flush;
    cv::cvtColor(rawFrame, newFrame, CV_RGBA2GRAY);
    if (framid == 0) {
      newFrame.copyTo(oldFrame);
      cx = oldFrame.cols / 2;
      cy = oldFrame.rows / 2;
      if (!FLAGS_undistort) {
        focal *= 2 * cx;
      }
      std::cout << "Cx is: " << cx << "\n";
      std::cout << "Cy is: " << cy << "\n";
      std::cout << "Focal is: " << focal << "\n";
      cv::goodFeaturesToTrack(oldFrame, corners_prev, maxCorners, qualityLevel,
                              minDistance, mask, blockSize, useHarrisDetector,
                              k);
      siftlatest = 0;
      siftids = std::vector<int>();
      for (int i = siftlatest; i < siftlatest + corners_prev.size(); i++) {
        siftids.push_back(i);
      }
      for (int i = 0; i < corners_prev.size(); i++) {
        colors.push_back(rawFrame.at<cv::Vec3b>(corners_prev[i]));
      }
      siftlatest = siftlatest + corners_prev.size();
      prevframe = framid;
      cv::imwrite(FLAGS_dirname + "/img_" + std::to_string(framid) + ".jpg",
                  rawFrame);
      framid++;
      continue;
    }

    // KL Tracker
    if (corners_prev.size() > 0) {
      calcOpticalFlowPyrLK(oldFrame, newFrame, corners_prev, corners, status,
                           err, winSize, 2, termcrit, 0, 0.001);

      calcOpticalFlowPyrLK(newFrame, oldFrame, corners, corners_inverse,
                           status_inverse, err, winSize, 2, termcrit, 0, 0.001);

      GetGoodPoints(corners_prev, corners_inverse, status, status_inverse);

      // Store Good Tracks;
      corr frame_corr(prevframe, framid);
      for (int i = 0; i < corners.size(); i++) {
        if (status[i]) {
          if ((corners[i].x > (2 * cx - 1)) || (corners[i].y > (2 * cy - 1)) ||
              (corners[i].x < 0) || (corners[i].y < 0)) {
            status[i] = 0;
            continue;
          }
          assert(corners[i].x <= 2 * cx - 1);
          assert(corners[i].y <= 2 * cy - 1);
          assert(corners[i].x >= 0);
          assert(corners[i].y >= 0);
          frame_corr.p1.push_back(corners_prev[i]);
          frame_corr.p2.push_back(corners[i]);
          frame_corr.unique_id.push_back(siftids[i]);
          frame_corr.col.push_back(colors[i]);
        }
      }
      CalculateDelta(frame_corr);
      all_corr.push_back(frame_corr);

      newFrame.copyTo(oldFrame);
      std::vector<int> newsiftids;
      corners_prev.clear();
      std::vector<cv::Vec3b> temp_colors;
      for (int i = 0; i < corners.size(); i++) {
        if (status[i]) {
          temp_colors.push_back(colors[i]);
          corners_prev.push_back(corners[i]);
          newsiftids.push_back(siftids[i]);
        }
      }
      colors = temp_colors;
      siftids = newsiftids;
      assert(siftids.size() == corners_prev.size());
      assert(siftids.size() == colors.size());
    }

    cv::imwrite(FLAGS_dirname + "/img_" + std::to_string(framid) + ".jpg",
                rawFrame);

    prevframe = framid;
    framid++;

    if ((framid % 5 == 0) || (corners_prev.size() < FLAGS_min_corners)) {
      mask = cv::Mat::ones(oldFrame.size(), CV_8UC1);
      cv::circle(mask, cv::Point2f(cx, cy), 1000, cv::Scalar(255), -1);
      for (int i = 0; i < corners_prev.size(); i++) {
        cv::circle(mask, corners_prev[i], 3, cv::Scalar(0), -1);
      }
      cv::imshow("mask", mask);
      std::vector<cv::Point2f> newcorners;
      cv::goodFeaturesToTrack(oldFrame, newcorners, maxCorners, qualityLevel,
                              minDistance, mask, blockSize, useHarrisDetector,
                              k);
      corners_prev.insert(corners_prev.end(), newcorners.begin(),
                          newcorners.end());
      for (int i = 0; i < newcorners.size(); i++) {
        colors.push_back(rawFrame.at<cv::Vec3b>(newcorners[i]));
      }
      for (int i = siftlatest; i < siftlatest + newcorners.size(); i++) {
        siftids.push_back(i);
      }
      siftlatest = siftlatest + newcorners.size();
      assert(colors.size() == siftids.size());
      assert(colors.size() == corners_prev.size());
    }

    for (int i = 0; i < corners_prev.size(); i++) {
      cv::circle(rawFrame, corners_prev[i], 4, cv::Scalar(0), -1);
    }
    cv::imshow("new", rawFrame);
    if (cv::waitKey(1) == 27) break;
  }

  std::cout << "Starting 1st round of compression\n";
  // Correspondance compression.
  int corres_skip = FLAGS_keyframe;
  std::vector<std::vector<corr> > Chunks;
  std::vector<std::vector<corr> > Chunks_Intermediate;
  Chunks_Intermediate.push_back(std::vector<corr>());
  int chunksize = FLAGS_chunks;
  std::vector<corr> compressed_all;
  int fid = 0;
  int chi = 1;
  for (int i = 0; i < all_corr.size();) {
    if (fid % chunksize == 0) {
      Chunks.push_back(std::vector<corr>());
      all_files.push_back(std::unordered_map<int, bool>());
      fileids.push_back(std::vector<int>());
    }
    corr compressed = all_corr[i];
    for (int j = 1; j < corres_skip && (i + j < all_corr.size()) &&
                    WithinRange(compressed);
         j++) {
      compressed = CompressCorr(compressed, all_corr[i + j]);
    }
    i = compressed.frame_2;

    if (FLAGS_corres) {
      ShowCorres(FLAGS_dirname, compressed);
    }
    required_images[compressed.frame_1] = false;
    required_images[compressed.frame_2] = false;

    if (all_files[Chunks.size() - 1].find(compressed.frame_1) ==
        all_files[Chunks.size() - 1].end())
      fileids[Chunks.size() - 1].push_back(compressed.frame_1);
    if (all_files[Chunks.size() - 1].find(compressed.frame_2) ==
        all_files[Chunks.size() - 1].end())
      fileids[Chunks.size() - 1].push_back(compressed.frame_2);
    all_files[Chunks.size() - 1][compressed.frame_1] = true;
    all_files[Chunks.size() - 1][compressed.frame_2] = true;
    compressed.frame_1 = fid;
    compressed.frame_2 = fid + 1;
    fid++;
    ChangeCenterSubtracted(compressed, cx, cy);
    if ((fid > chi * chunksize - windows_size) and
        (fid < chi * chunksize + windows_size)) {
      Chunks_Intermediate[Chunks_Intermediate.size() - 1].push_back(compressed);
    } else if (fid == chi * chunksize + windows_size) {
      chi++;
      Chunks_Intermediate.push_back(std::vector<corr>());
    }
    Chunks[Chunks.size() - 1].push_back(compressed);
  }
  std::cout << "Done with 1st round of compression\n";
  std::ofstream listfocalglobal, inifile;
  listfocalglobal.open(FLAGS_dirname + "/list_focal.txt");
  inifile.open(FLAGS_dirname + "/clusternames.ini");

  for (int ch = 0; ch < Chunks.size(); ch++) {
    all_corr = Chunks[ch];

    system(("mkdir " + FLAGS_dirname + "/batch_" + std::to_string(ch)).c_str());
    std::ofstream corresfile, rdata, tdata, edata, pdata, listfocal, list_focal,
        num_cors;
    rdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/R5point.txt");
    tdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/T5point.txt");
    edata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) + "/E5point.txt");
    pdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) +
               "/original_pairs5point.txt");
    listfocal.open(FLAGS_dirname + "/batch_" + std::to_string(ch) +
                   "/listsize_focal1.txt");
    list_focal.open(FLAGS_dirname + "/batch_" + std::to_string(ch) +
                    "/list_focal.txt");
    num_cors.open(FLAGS_dirname + "/batch_" + std::to_string(ch) +
                  "/num_cors.txt");
    FILE *fp = fopen((FLAGS_dirname + "/batch_" + std::to_string(ch) +
                      "/matches_forRtinlier5point.txt")
                         .c_str(),
                     "w");
    fprintf(fp, "                                  \n");

    std::vector<corr> new_compressed;
    for (int i = 0; i < all_corr.size(); i++) {
      corr compressed = all_corr[i];
      new_compressed.push_back(compressed);
      for (int j = 1; j < FLAGS_overlap && (i + j < all_corr.size()); j++) {
        compressed = CompressCorr(compressed, all_corr[i + j]);
        new_compressed.push_back(compressed);
      }
    }
    std::unordered_map<int, int> sift_count;
    std::unordered_map<int, std::unordered_set<int> > sift_frames;
    all_corr = new_compressed;
    std::cout << "Done with 2nd round of compression\n";
    numoutmatches = 0;
    for (int i = 0; i < all_corr.size(); i++) {
      TwoViewInfo twoview_info;
      std::vector<int> inliers;
      if (GetEssentialRT(all_corr[i], twoview_info, inliers, options, focal)) {
        numoutmatches++;
        rdata << twoview_info.rotationmat_2(0, 0) << " "
              << twoview_info.rotationmat_2(0, 1) << " "
              << twoview_info.rotationmat_2(0, 2) << " "
              << twoview_info.rotationmat_2(1, 0) << " "
              << twoview_info.rotationmat_2(1, 1) << " "
              << twoview_info.rotationmat_2(1, 2) << " "
              << twoview_info.rotationmat_2(2, 0) << " "
              << twoview_info.rotationmat_2(2, 1) << " "
              << twoview_info.rotationmat_2(2, 2) << "\n";
        tdata << twoview_info.translation_2(0) << " "
              << twoview_info.translation_2(1) << " "
              << twoview_info.translation_2(2) << "\n";
        edata << twoview_info.essential_mat(0, 0) << " "
              << twoview_info.essential_mat(0, 1) << " "
              << twoview_info.essential_mat(0, 2) << " "
              << twoview_info.essential_mat(1, 0) << " "
              << twoview_info.essential_mat(1, 1) << " "
              << twoview_info.essential_mat(1, 2) << " "
              << twoview_info.essential_mat(2, 0) << " "
              << twoview_info.essential_mat(2, 1) << " "
              << twoview_info.essential_mat(2, 2) << "\n";
        pdata << all_corr[i].frame_1 + 1 - chunksize * ch << " "
              << all_corr[i].frame_2 + 1 - chunksize * ch
              << " 1.00000 1.00000\n";
        fprintf(fp, "%d %d %ld\n", all_corr[i].frame_1 - chunksize * ch,
                all_corr[i].frame_2 - chunksize * ch, inliers.size());
        for (int j = 0; j < inliers.size(); j++) {
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
          fprintf(fp, "%d %f %f %d %f %f\n", all_corr[i].unique_id[loc],
                  all_corr[i].p1[loc].x, all_corr[i].p1[loc].y,
                  all_corr[i].unique_id[loc], all_corr[i].p2[loc].x,
                  all_corr[i].p2[loc].y);
        }
      } else {
        std::cerr << "Something bad still happened!!!!!!!!!!\n";
      }
    }
    for (int i = 0; i < fileids[ch].size(); i++) {
      listfocal << "img_" << fileids[ch][i] << ".jpg"
                << " 0 " << focal << "\n";
      list_focal << "img_" << fileids[ch][i] << ".jpg " << focal << "\n";
    }
    for (int i = 0; i < fileids[ch].size() - 1; i++) {
      listfocalglobal << "img_" << fileids[ch][i] << ".jpg " << focal << "\n";
    }
    int total_size = 0;
    int count = 0;
    std::vector<int> counts;
    for (auto it : sift_count) {
      counts.push_back(it.second);
      count += it.second;
      total_size++;
    }
    std::nth_element(counts.begin(), counts.begin() + total_size / 2,
                     counts.end());
    std::cout << "Mean: " << count / total_size
              << " Median: " << counts[total_size / 2] << "\n";
    num_cors << counts[total_size / 2] << "\n";
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

  std::cout << "Number of intermedia chunks : " << Chunks_Intermediate.size()
            << "\n";
  for (int ch = 0; ch < Chunks_Intermediate.size(); ch++) {
    std::ofstream rdata;
    rdata.open(FLAGS_dirname + "/batch_" + std::to_string(ch) +
               "/IntermediateRT.txt");
    all_corr = Chunks_Intermediate[ch];
    std::vector<corr> new_compressed;
    for (int i = 0; i < all_corr.size(); i++) {
      corr compressed = all_corr[i];
      new_compressed.push_back(compressed);
      for (int j = 1; i + j < all_corr.size(); j++) {
        compressed = CompressCorr(compressed, all_corr[i + j]);
        new_compressed.push_back(compressed);
      }
    }
    all_corr = new_compressed;
    std::cout << "Done with 3nd round of compression\n";
    numoutmatches = 0;
    for (int i = 0; i < all_corr.size(); i++) {
      // std::cerr << "Processing " << i << " out of " << all_corr.size() <<
      // "\n";
      TwoViewInfo twoview_info;
      std::vector<int> inliers;
      if (GetEssentialRT(all_corr[i], twoview_info, inliers, options, focal)) {
        rdata << all_corr[i].frame_1 << " " << all_corr[i].frame_2 << "\n";
        rdata << twoview_info.rotationmat_2(0, 0) << " "
              << twoview_info.rotationmat_2(0, 1) << " "
              << twoview_info.rotationmat_2(0, 2) << " "
              << twoview_info.rotationmat_2(1, 0) << " "
              << twoview_info.rotationmat_2(1, 1) << " "
              << twoview_info.rotationmat_2(1, 2) << " "
              << twoview_info.rotationmat_2(2, 0) << " "
              << twoview_info.rotationmat_2(2, 1) << " "
              << twoview_info.rotationmat_2(2, 2) << "\n";
        rdata << twoview_info.translation_2(0) << " "
              << twoview_info.translation_2(1) << " "
              << twoview_info.translation_2(2) << "\n";

        rdata << inliers.size() << "\n";
        for (int j = 0; j < inliers.size(); j++) {
          int loc = inliers[j];
          int sftid = all_corr[i].unique_id[loc];
          int f1 = all_corr[i].frame_1;
          int f2 = all_corr[i].frame_2;
          rdata << (int)all_corr[i].col[loc].val[0] << " "
                << (int)all_corr[i].col[loc].val[1] << " "
                << (int)all_corr[i].col[loc].val[2] << " "
                << all_corr[i].unique_id[loc] << " " << all_corr[i].p1[loc].x
                << " " << all_corr[i].p1[loc].y << " "
                << all_corr[i].unique_id[loc] << " " << all_corr[i].p2[loc].x
                << " " << all_corr[i].p2[loc].y << "\n";
        }
      } else {
        std::cerr << "Something bad still happened!!!!!!!!!!\n";
      }
    }
    rdata.close();
  }

  listfocalglobal << "img_" << *((*fileids.rbegin()).rbegin()) << ".jpg "
                  << focal << "\n";
  listfocalglobal.close();
  inifile.close();

  if (FLAGS_clear_non_kf) {
    for (auto it : required_images) {
      if (it.second) {
        system(("rm -f " + FLAGS_dirname + "/img_" + std::to_string(it.first) +
                ".jpg")
                   .c_str());
      }
    }
  }

  return 0;
}
