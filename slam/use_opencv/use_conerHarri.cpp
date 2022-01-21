#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  // 检测角点
   Mat src = imread("../image/a.jpg", 0);
   imshow("src",src);
   Mat conerStrength;
   conerHarris(src, conerStrength, 2, 3, 0.01);
   Mat harrisConer;
   threshold(conerStrength, conerHarris, 0.00001, 255, THRESH_BINARY);
   imshow("焦点检测", harrisConer);
   waiKey();
   return 0;
}
