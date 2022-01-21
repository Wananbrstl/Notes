#include<iostream>
#include "opencv4/opencv2/opencv.hpp"

using namespace cv;
int main(int argc, char *argv[])
{
    // -------------------[创建矩阵]-----------------
    // Mat m1 = Mat::zeros(Size(400,400), CV_8UC3);
    // // 1. 直接通过重载
    // Mat m2 = m1;
    // imshow("m1", m1);
    // m2 = 255;
    // imshow("前拷贝 m1", m1);
    // // 2. 深拷贝
    // Mat m3;
    // m1.copyTo(m3);
    // Mat m4 = m1.clone();
    // m4 = Scalar(255,252,252);
    // imshow(" clone m1 ", m1);

    //---------------[腐蚀操作]-------------
    // 使用暗色部分去 ”腐蚀“ 高亮部分 
    // Mat srcimag = imread("../image/a.jpg");
    // imshow("srcimag", srcimag); 
    // Mat element = getStructuringElement(MORPH_RECT, Size(15,15));
    // Mat dstImag;
    // erode(srcimag, dstImag, element);
    // // 显示腐蚀后的效果
    // imshow("腐蚀后的效果", dstImag);

    // // --------------[图像模糊]-------------
    // // 均值滤波操作， 让一张图像变得模糊
    // Mat dstImag2;
    // blur(srcimag,dstImag2,Size(7,7));
    // imshow("均值滤波【效果图】", dstImag2);

    // // --------【边缘检测】------------
    // // 首先转化成灰度图，使用blur函数进行降噪, 然后再canny边缘检测算法
    // Mat dstImag3, edget, grayImag;
    // cvtColor(srcimag,grayImag, COLOR_BGR2GRAY);
    // blur(grayImag, edget, Size(3,3) );
    // Canny(edget, dstImag3, 3, 9, 3);
    // imshow("Canny边缘检测", dstImag3);
    // waitKey(0);

    //---------------------【融合图片】---------------
    Mat lolLogo = imread("../image/lol.jpg");
    Mat anni = imread("../image/anni.jpg");
    imshow("lolLogo", lolLogo);
    imshow("anni", anni);
    Mat imageROI;
    imageROI = anni(Rect(800, 350, lolLogo.cols, lolLogo.rows));
    addWeighted(imageROI, 0.5, lolLogo, 0.3, 0., imageROI);
    imshow("[元化] + 【lgoo】", imageROI);
    waitKey(0);
    return 0;
}
