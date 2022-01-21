#include <iostream>
#include <chrono>
using namespace std;

#include"opencv4/opencv2/core/core.hpp"
#include"opencv4/opencv2/highgui/highgui.hpp"

int main(int argc, char const *argv[])
{
    cv::Mat image;
    image = cv::imread("../image/a.jpg");
    if(image.data == nullptr)
    {
        cerr << " file "<<"don't exit" << endl;
        return 0;
    }
    cout << "图像的宽度为: " << image.cols << ", 高为：" << image.rows << "通道为： " << image.channels() << endl;
    cv::imshow("image", image);
    cv::waitKey(0);
    //if(image.type() != CV_8UC1 && image.type() != CV_UC3)
    //{
     //   cout << "请输入一张"
    // }
    return 0;
}
