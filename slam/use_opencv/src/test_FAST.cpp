#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char const* argv[])
{
  Mat srcImage = imread("../image/a.jpg");
  Mat src;
  cvtColor(srcImage, src, COLOR_BGR2GRAY);
  if(src.empty()){
    cout << "src is empty ! " << endl;
    return 1;
  } 
  vector<KeyPoint> keypoints;
  Ptr<FastFeatureDetector> FASTdetect = FastFeatureDetector::create(40);
  FASTdetect->setType(FastFeatureDetector::TYPE_9_16);
  FASTdetect->detect(src, keypoints);
  
  Mat res;
  drawKeypoints(srcImage, keypoints, res, Scalar(255,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  imshow("result", res);
  waitKey();
  return 0;
}
