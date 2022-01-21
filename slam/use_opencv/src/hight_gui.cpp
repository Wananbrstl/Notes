#include<iostream>
#include"opencv4/opencv2/highgui.hpp"
#include"opencv4/opencv2/imgcodecs.hpp"

using namespace std;
using namespace cv;

Mat src1;
Mat src2;
Mat dst;

const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;

static void on_trackbar(int , void*)
{
    alpha = (double)alpha_slider/alpha_slider_max;
    beta = 1.0 - alpha;
    addWeighted(src1, alpha, src2, beta, 0.0, dst);
    imshow("line Blend", dst);
}

int main(void )
{
    src1 = imread("../image/LinuxLogo.jpg");
    src2 = imread("../image/WindowsLogo.jpg");
    if(src1.empty()){
        cout << "Error Loading src1 \n";
        return -1;
    }
    if(src2.empty()){
        cout << "Error Loading src2 \n";
        return -1;
    }
    cout << "ready" << endl;
    alpha_slider = 0;
    namedWindow("Line Blend", WINDOW_AUTOSIZE);
    char TrackbarName[50];
    sprintf(TrackbarName, "Alpha x %d", alpha_slider_max);
    createTrackbar(TrackbarName, "Line Blend", &alpha_slider, alpha_slider_max, on_trackbar);

    on_trackbar(alpha_slider, 0);
    waitKey(0);
    return 0;
}
