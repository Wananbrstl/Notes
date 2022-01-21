/*
    opencv的一些基础操作
*/
#include <iostream>
#include <string>
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"
using namespace std;

int main(int argc, char *argv[])
{
    // 创建一个图像
    cv::Mat image;
    image = cv::imread("../image/a.jpg");
    
    // 浅拷贝
    cv::Mat copyImage(image);
    cv::Mat grayImage = cv::imread("../image/a.jpg",cv::IMREAD_GRAYSCALE);
    cv::Mat hvsImage = cv::imread("../image/a.jpg",cv::IMREAD_UNCHANGED);

    cv::imshow("image",image);
    cv::imshow("grayImage", grayImage);
    cv::imshow("hvsImag",hvsImage);
    cv::waitKey(0);    
    return 0;
}
