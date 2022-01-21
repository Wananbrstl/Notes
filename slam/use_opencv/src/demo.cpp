#include "demo.h"
#include "opencv4/opencv2/highgui.hpp"

using namespace std;
using namespace cv;

#define WINDOW_NAME "【线性混合实例】"

const int g_nMaxAlphaValue = 100;
int g_nAlphaValueSlider;
double g_dAlphaValue;
double g_dBetaValue;

Mat g_srcImage1;
Mat g_srcImage2;
Mat g_dstImage;

void on_TrackBar(int , void *)
{
    g_dAlphaValue = (double) g_nAlphaValueSlider / g_nMaxAlphaValue;
    g_dBetaValue = 1 - g_dAlphaValue;

    addWeighted(g_srcImage1, g_dAlphaValue, g_srcImage2, g_dBetaValue, 0.0, g_dstImage);
    imshow(WINDOW_NAME, g_dstImage);
}
void demo::use_TrackBar()
{
    g_srcImage1 = imread("../image/lolLogo.jpg");
    g_srcImage2 = imread("../image/anni.jpg");

    if(!g_srcImage1.data)
    {
        printf("读取错误");
    }
    if(!g_srcImage2.data)
    {
        printf("图像二读取错误");
    }

    g_nAlphaValueSlider = 70;

    namedWindow(WINDOW_NAME, 1);
    char TrackbarName[50];
    sprintf(TrackbarName, "透明度%d", g_nAlphaValueSlider);
    createTrackbar(TrackbarName, WINDOW_NAME, &g_nAlphaValueSlider, g_nMaxAlphaValue, on_TrackBar);

    on_TrackBar(g_nAlphaValueSlider, 0);
    waitKey(0);
}

demo::demo(/* args */)
{
}

demo::~demo()
{
}

void demo::showImage()
{
    Mat image = imread("../image/a.jpg");
    imshow("image", image);
    waitKey(0);
}
void demo::showDataStructure()
{
    // Mat数据结构
    // Mat m = (Mat_<double>)
    // 点的数据结构
    Point2d point1(1,3);
    cout << "[point]         " << point1 <<endl;
    // 颜色的定义
    Scalar color;
    color.
}

void drawEllipse(Mat img, double angle)
{
     int thickness = 2;
     int lineType = cv::LINE_8;

     ellipse(img, 
            ,Point(img.))
}