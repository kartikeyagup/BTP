#include "sifthelpers.h"

struct sift_corr Running_SIFT(std::string file1, std::string file2) {
  SiftGPU sift;
  char * test [] = {"-fo","-1","-da","-v","0","-p","5184x3456","-b","-nomc"};
  sift.ParseParam(9, test);

  int support  = sift.CreateContextGL();
  if(support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {
    struct sift_corr temp;
    return temp;
  } 
  sift.RunSIFT(file1.c_str());
  // sift.SaveSIFT("img1.sift");
  int num1 = sift.GetFeatureNum();

  std::vector<float> des1(128*num1);
  std::vector<SiftGPU::SiftKeypoint> keys1(num1);

  sift.GetFeatureVector(&keys1[0], &des1[0]);

  sift.RunSIFT(file2.c_str());
  // sift.SaveSIFT("img2.sift");
  int num2 = sift.GetFeatureNum();

  std::cout << "number of matches   : "<< num1 << "  " << num2 << std::endl;

  std::vector<float> des2(128*num2);
  std::vector<SiftGPU::SiftKeypoint>keys2(num2);
  sift.GetFeatureVector(&keys2[0], &des2[0]);

  SiftMatchGPU matcher(4096);
  matcher.VerifyContextGL();

  matcher.SetDescriptors(0, num1, &des1[0]); //image 1
  matcher.SetDescriptors(1, num2, &des2[0]); //image 2

  int (*match_buf)[2] = new int[num1][2];

  int num_match = matcher.GetSiftMatch(num1, match_buf);


  struct sift_corr answer;

  std::unordered_map<struct sift_point, bool> first_img;
  std::unordered_map<struct sift_point, bool> second_img;

  for(int i = 0 ; i < num_match ; i++) {
    SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];//feature point of first image
    SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];//corresponding feature point of second image
    struct sift_point f1;
    f1.x = (int)key1.x;
    f1.y = (int)key1.y;
    struct sift_point f2;
    f2.x = (int)key2.x;
    f2.y = (int)key2.y;
    if(first_img.count(f1) == 0 && second_img.count(f2) == 0) {
      answer.first_img.push_back(keys1[match_buf[i][0]]);
      answer.second_img.push_back(keys2[match_buf[i][1]]);
      first_img[f1] = true;
      second_img[f2] = true;
    }
  }
  assert(answer.first_img.size() == answer.second_img.size());
  std::cout << answer.first_img.size() << "   sift matches were found;\n";

  return answer;
}

std::vector<std::pair<cv::Point2f, cv::Point2f> > RunSift(std::string f1, std::string f2, cv::Point2f center) {
  sift_corr res = Running_SIFT(f1, f2);
  return res.GetCSCorrs(center);
}

