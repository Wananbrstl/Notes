#include<iostream>
#include<opencv2/opencv.hpp>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include"Eigen/Core"
#include"Eigen/Geometry"
#include<g2o/core/block_solver.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<chrono>

using namespace std;
using namespace cv;

// 特征匹配
void find_feature_match(const Mat& img_1,
                        const Mat& img_2,
                        vector<KeyPoint>& keypoints_1,
                        vector<KeyPoint>& keypoints_2,
                        vector<DMatch>& matches);

// 相机归一化
Point2d pixel2cam(const Point2d& p, const Mat& K);

// BA
void bundleAdjustment(const vector<Point3f> points_3d, 
                      const vector<Point2f> points_2d,
                      const Mat& K, 
                      Mat& R, Mat& t);

int main(int argc, char** argv)
{
  // 1. 读取照片
  if(argc != 4 ) {
    cout << "请输入三张图片, 最后一张为深度图像" << endl;
    return 1;
  }
  Mat img_1 = imread(argv[1]);
  Mat img_2 = imread(argv[2]);
  Mat d1 = imread(argv[3], CV_16SC1);
  if(img_1.empty() || img_2.empty() || d1.empty()) {
    cout << "请输入正确地址" << endl;
    return 1;
  }

  // 2. 匹配关系
  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_match(img_1, img_2, keypoints_1, keypoints_2, matches);

  // 3. 建立3D点
  // 内参矩阵
  Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  vector<Point3f> pts_3d;
  vector<Point2f> pts_2d;
  for(DMatch m: matches)
  {
    ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y]))[int(keypoints_1[m.queryIdx].pt.x)];
    if(d == 0){continue;}
    float dd = d / 5000.0;
    Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd , dd));
    pts_2d.push_back(keypoints_2[m.trainIdx].pt);
  }
  cout << "-- 3d-2d pairs: " << pts_3d.size() << endl;

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  // 计算
  Mat r, t;
  solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
  Mat R;
  cv::Rodrigues(r, R); // 使用Rodrigues公式
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "-- solve pnp in opencv cost time :" << time_used.count << " seconds." << endl;

  cout << "-- R = " << endl << R << endl;
  cout << "-- t = " << endl << t << endl;

  // 优化BA

  return 0;
}
