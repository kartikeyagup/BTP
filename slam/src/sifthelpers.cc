#include "sifthelpers.h"

void RunAndSaveSift(std::vector<std::string> paths) {
  std::vector<std::pair<cv::Point2f, cv::Point2f> > answer;

  SiftGPU sift;
  
  char * test [] = {"-fo","-1","-da","-v","0","-p","5184x3456","-b","-nomc"};
  sift.ParseParam(9, test);

  int support  = sift.CreateContextGL();
  if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
    return;
  }

  for (auto it:paths) {
    if (sift.RunSIFT((it).c_str())) {
      sift.SaveSIFT((it+".sift").c_str());
    }
  }
}

std::vector<std::pair<cv::Point2f, cv::Point2f> > RunSift(std::string file1, std::string file2, cv::Point2f center) {
  std::vector<std::pair<cv::Point2f, cv::Point2f> > answer;
  SiftGPU sift;
  
  char * test [] = {"-fo","-1","-da","-v","0","-p","5184x3456","-b","-nomc"};
  sift.ParseParam(9, test);

  int support  = sift.CreateContextGL();
  if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
    return answer;
  }

  if (!sift.ReadSIFT((file1+".sift").c_str())) {
    assert(false);
  }
  int num1 = sift.GetFeatureNum();

  std::vector<float> des1(128*num1);
  std::vector<SiftGPU::SiftKeypoint> keys1(num1);
  sift.GetFeatureVector(&keys1[0], &des1[0]);

  if (!sift.ReadSIFT((file2+".sift").c_str())) {
    assert(false);
  }
  int num2 = sift.GetFeatureNum();

  std::vector<float> des2(128*num2);
  std::vector<SiftGPU::SiftKeypoint> keys2(num2);
  sift.GetFeatureVector(&keys2[0], &des2[0]);

  SiftMatchGPU matcher(4096);
  matcher.VerifyContextGL();

  matcher.SetDescriptors(0, num1, &des1[0]); //image 1
  matcher.SetDescriptors(1, num2, &des2[0]); //image 2

  int (*match_buf)[2] = new int[num1][2];

  int num_match = matcher.GetSiftMatch(num1, match_buf);

  for(int i = 0 ; i < num_match ; i++) {
    SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];//feature point of first image
    SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];//corresponding feature point of second image    struct sift_point f1;
    std::pair<cv::Point2f, cv::Point2f> temp;
    temp.first.x = key1.x - center.x;
    temp.first.y = key1.y - center.y;
    temp.second.x = key2.x - center.x;
    temp.second.y = key2.y - center.y;
    answer.push_back(temp);
  }
  std::cout << answer.size() << "   sift matches were found;\n";
  return answer;
}
