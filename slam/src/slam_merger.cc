#include "slam_merger.h"

void slam_merger::join_sets(std::vector<int> frames1, 
    std::vector<int> frames2,
    std::vector<Corr3D> &corr1, std::vector<Corr3D> &corr2) {
  // Fix old sift ids
  // Find sift correspondances
  // Triangulate
  // Obtain RST from the correspondances
  // Apply RST on nvm2

  int siftoffset = 1 + f1.get_max_sift();
  for (int i=0; i<f2.corr_data.size(); i++) {
    f2.corr_data[i].ChangeSiftOffset(siftoffset);
  }

  int maxsftid = 1 + std::max(f1.get_max_sift(), f2.get_max_sift());
  std::vector<std::pair<imgcorr, imgcorr> > all_matches;
  corr1.clear(); corr2.clear();
  for (int i=0; i<frames1.size(); i++) {
    for (int j=0; j<frames2.size(); j++) {
      std::vector<std::pair<cv::Point2f, cv::Point2f> > corrs = 
        RunSift(
          p1 + f1.kf_data[frames1[i]].filename, 
          p2 + f2.kf_data[frames2[j]].filename, 
          f1.getCenter());
      for (auto it: corrs) {
        // Correlate and form corrs in corr1 and corr2
        // Assign unique siftids.
        // CHECK UNIQUE ID
        bool done = false;
        for (int k=0; k<corr1.size(); k++) {
          if (corr1[k].belongs(it.first, frames1[i])) {
            corr2[k].AddNew(it.second, frames2[j]);
            done = true;
            break;
          } else if (corr2[k].belongs(it.second, frames2[j])) {
            corr1[k].AddNew(it.first, frames1[i]);
            done = true;
            break;
          }
        } 
        if (!done) { 
          Corr3D t1, t2;
          t1.color = f1.getColorCS(frames1[i], it.first);
          t2.color = f2.getColorCS(frames2[j], it.second);
          t1.corr.push_back(imgcorr(frames1[i], maxsftid, it.first));
          t2.corr.push_back(imgcorr(frames2[j], maxsftid, it.second));
          maxsftid++;
          corr1.push_back(t1);
          corr2.push_back(t2);
        }
      }
    }
  }
  assert(corr1.size() == corr2.size());

  int total(0), value(0);
  // Triangulate all corr's
  for (int i=0; i<corr1.size(); i++) {
    if ((corr1[i].numpoints()>=2) && (corr2[i].numpoints()>=2)) {
      // Check Fundamental Matrix over all corrs
      total++;
      value += corr1[i].numpoints();
      // int wdt = f1.images[0].cols;
      // int ht = f1.images[0].rows;
      // cv::Mat im3(ht, wdt*corr1[i].corr.size(), CV_8UC3);
      // for (int k=0; k<corr1[i].corr.size(); k++) {
      //   cv::Mat img(im3, cv::Rect(k*wdt, 0, wdt, ht));
      //   f1.images[corr1[i].corr[k].imgid].copyTo(img);
      //   cv::circle(img, f1.getCenter() + corr1[i].corr[k].img_location, 5, cv::Scalar(255,0,0), -1);
      // }
      // cv::imshow("batch left", im3);
      // cv::waitKey(0);
      // cv::Mat im4(sz1.height, sz1.width+sz2.width, CV_8UC1);
      Triangulate_Internally(corr1[i], f1.kf_data);
      Triangulate_Internally(corr2[i], f2.kf_data);
      f1.corr_data.push_back(corr1[i]);
      f2.corr_data.push_back(corr2[i]);
    }
  }
  std::cout << value << " Points in " << total << "matches\n";
  // Obtain and apply RST
  GetBestRST(f1, f2);
}

void slam_merger::merge_and_save(std::string path) {
  // Run join sets on subset of images
  // Migrate old points
  // Add common points to it
  // Save new merged files to disk
  std::vector<int> frames1, frames2;
  int frameskip = 5;
  for (int i=0; i<f1.num_frames()/frameskip; i++) {
    frames1.push_back(frameskip*i);
  }
  for (int i=0; i<f2.num_frames()/frameskip; i++) {
    frames2.push_back(frameskip*i);
  }
  std::vector<Corr3D> c1, c2;

  join_sets(frames1, frames2, c1, c2);
  assert(c1.size() == c2.size());

  // Saving images at path
  for (int i=0; i<f1.images.size(); i++) {
    cv::imwrite(path + "0_"+f1.kf_data[i].filename, f1.images[i]);
  }
  for (int i=0; i<f2.images.size(); i++) {
    cv::imwrite(path + "1_"+f2.kf_data[i].filename, f2.images[i]);
  }

  // Fix image names
  for (int i=0; i<f1.kf_data.size(); i++) {
    f1.kf_data[i].filename = "0_" + f1.kf_data[i].filename;
  }
  for (int i=0; i<f2.kf_data.size(); i++) {
    f2.kf_data[i].filename = "1_" + f2.kf_data[i].filename;
  }

  // Combine keyframes
  for (int i=0; i<f2.kf_data.size(); i++) {
    f1.kf_data.push_back(f2.kf_data[i]);
  }
  
  // Combine all 
  std::unordered_map<int, int> sift_to_pos;
  int offset = f1.kf_data.size();
  for (int i=0; i<f1.corr_data.size(); i++) {
    assert(sift_to_pos.find(f1.corr_data[i].getSiftId()) == sift_to_pos.end());
    sift_to_pos[f1.corr_data[i].getSiftId()] = i;
  }

  for (int i=0; i<f2.corr_data.size(); i++) {
    if (sift_to_pos.find(f2.corr_data[i].getSiftId()) == sift_to_pos.end()) {
      // New sift id ie normal point
      f1.corr_data.push_back(fixCorr3D(f2.corr_data[i], offset));
    } else {
      // Old point so only add its correspondance information
      int corr_index = sift_to_pos[f2.corr_data[i].getSiftId()];
      for (auto it: f2.corr_data[i].corr) {
        f1.corr_data[corr_index].corr.push_back(fix_corr(it, offset));
      }
    }
  }
  f1.save_ply_file(path + "outputMergednvm.ply");
}
