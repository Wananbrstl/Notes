#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>


using namespace std;

int main(int argc, char** argv)
{
  cv::Mat src = cv::imread("../image/a.jpg");

  // 创建一个特征检测
  cv::Ptr<cv::SURF> SURFdetector = cv::SURF::create(40);
  vector<cv::KeyPoint> keypoints;
  SURFdetector->detect(src, keypoints);

  // drawKeypoints
  cv::Mat res;
  cv::drawKeypoints(src, keypoints, res);
  cv::imshow("res", res);
  cv::waitKey();
  return 0 ;
}
