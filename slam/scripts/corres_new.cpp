#include <iostream>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include <fstream>

DEFINE_string(matches_file, "matches_forRtinlier5point.txt", "Path to matches file");
DEFINE_string(map_path, "cluster_list_map.txt", "Path to the cluster list file");
DEFINE_string(world_path, "world.txt", "Path to the output world file");
DEFINE_string(list_focal_file, "listfocal.txt", "Path to the list focal file");
DEFINE_string(data_dir, "data0", "Path where images are present");
DEFINE_int32(num_camera, 0, "Number of cameras");

struct imgcorr {
  int siftid;
  cv::Point2f imglocation;

  imgcorr() {};

  imgcorr(int sift, float p1, float p2) {
    siftid = sift;
    imglocation.x = p1;
    imglocation.y = p2;    
  }
};

struct world_corr {
  cv::Point3f location;
  cv::Vec3b color;
  // img id to match
  std::unordered_map<int, imgcorr> pairs;

  world_corr() {};
};

struct total_world {
  // sift id to matches
  std::unordered_map<int, world_corr> all_matches;

  total_world() {};

  total_world(std::string data_dir, std::string matches_file,
    std::string cluster_path) {
    std::ifstream cluster;
    cluster.open(cluster_path);
    std::vector<int> correct_frame_id;
    int temp;
    while (cluster >> temp) {
      correct_frame_id.push_back(temp);
    }
    cluster.close();

    std::ifstream matches;
    matches.open(matches_file);
    int num_pairs;
    matches >> num_pairs;
    for (int i=0; i<num_pairs; i++) {
      int f1, f2, numc;
      matches >> f1 >> f2 >> numc;
      assert(f1 < correct_frame_id.size());
      assert(f2 < correct_frame_id.size());
      f1 = correct_frame_id[f1];
      f2 = correct_frame_id[f2];
      for (int j=0; j< numc; j++) {
        int sf1, sf2;
        float p1x, p1y, p2x, p2y;
        matches >> sf1 >> p1x >> p1y >> sf2 >> p2x >> p2y;
        if (sf1!=sf2) {
          std::cerr << "Sift IDs did not match\n";
          assert(sf1==sf2);
        }
        if (all_matches.find(sf1) == all_matches.end()) {
          all_matches[sf1] = world_corr();
          all_matches[sf1].location.x = 0;
          all_matches[sf1].location.y = 0;
          all_matches[sf1].location.z = 0;
          // Put in color
        }
        all_matches[sf1].pairs[f1] = imgcorr(sf1, p1x, p1y);
        all_matches[sf2].pairs[f2] = imgcorr(sf2, p2x, p2y);
      }
    }
    matches.close();    
  }

  void writeWorld(std::string path) {
    std::ofstream output;
    output.open(path);
    for (auto it: all_matches) {
      output << it.second.location.x << " " << it.second.location.y << " " << it.second.location.z << " ";
      output << (int) it.second.color[0] << " " << (int) it.second.color[1] << " " << (int) it.second.color[2] << " "; 
      output << it.second.pairs.size() << " ";
      for (auto it1 : it.second.pairs) { 
        output << it1.first << " " << it1.second.siftid << " ";
        output << it1.second.imglocation.x << " " << it1.second.imglocation.y << " ";
      }
      output << "\n";
    }
    output.close();
  }
};

int main(int argc, char **argv)
{
  google::SetUsageMessage("dense --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);
 
  total_world input(FLAGS_data_dir, FLAGS_matches_file, FLAGS_map_path);
  input.writeWorld(FLAGS_world_path);

  std::cout << "Entered main of corres\n";
  return 0;
}