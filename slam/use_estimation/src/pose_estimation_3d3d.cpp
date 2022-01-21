/*
 * 3d-3d ICP问题估计
 */
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <chrono>
#include <sophus/se3.h>

using namespace std;
using namespace cv;

// 两张图像的匹配点
void find_feature_matches(const Mat& img1,
                          const Mat& img2, 
                          vector<KeyPoint> &keypoints_1,
                          vector<KeyPoint> &keypoints_2,
                          vector<DMatch> &matches);

// 坐标转化 pixel -> cam
Point2d pixel2cam(const Point2d& p, const Mat& K);

// R, t估计(ICP问题) 1.SVD方法 2. BA方法
void pose_estimation_3d3d(const vector<Point3f>& p1, 
                          const vector<Point3f>& p2, 
                          Mat &R, Mat &t);

// BA优化
void bundleAdjustment(const vector<Point3f>& p1, 
                      const vector<Point3f>& p2,
                      Mat &R, Mat &t);

// BA问题中边和顶点

int main(int argc, char const* argv[])
{
  // 1. 读取照片 
  Mat src1 = imread("../images/1.png");
  Mat src2 = imread("../images/2.png");
  if(src1.empty() || src2.empty()){
    cout << "[read error] --Please input the right images address" << endl;
    return 1;
  }

  // 2. 提取特征
  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(src1, src2, keypoints_1, keypoints_2, matches);
  cout << " matches has : " << matches.size() << endl;

  // 3. 建立3D点
  Mat depth1 = imread("../images/1_depth.png", IMREAD_ANYDEPTH);
  Mat depth2 = imread("../images/2_depth.png", IMREAD_ANYDEPTH);
  //Mat depth1 = imread("../images/1_depth.png");
  //Mat depth2 = imread("../images/2_depth.png");
  cout << "[depth image information] --<channel>: " <<depth1.channels() <<endl;
  if(depth1.empty() || depth2.empty()){
    cout << "[ERROR] --Please Input The Right FileName" << endl;
    return 1;
  }
  
  vector<Point3f> pts1, pts2;
  // 相机的内参矩阵
  Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  for(DMatch m: matches)
  {
    ushort d1 = depth1.ptr<ushort>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
    ushort d2 = depth2.ptr<ushort>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
    //ushort d1 = depth1.at<ushort>(int(keypoints_1[m.queryIdx].pt.x), int(keypoints_1[m.queryIdx].pt.y));
    //ushort d2 = depth2.at<ushort>(int(keypoints_2[m.trainIdx].pt.x), int(keypoints_2[m.trainIdx].pt.y));

    if(d1 == 0 || d2 == 0) continue;
    Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    Point2d p2 = pixel2cam(keypoints_2[m.queryIdx].pt, K);
    float dd1 = float(d1) / 5000.0;
    float dd2 = float(d2) / 5000.0;
    pts1.push_back(Point3f(p1.x * dd1, p1.y *dd1, dd1)); 
    pts2.push_back(Point3f(p2.x * dd2, p2.y *dd2, dd2)); 
  }

  cout << "3d-3d paris has:  " << pts1.size() << endl;

  // ICP问题求解
  Mat R, t;
  pose_estimation_3d3d(pts1, pts2, R, t);
  cout << "--ICP via SVD results: " << endl;
  cout << "--R = " << endl << R << endl;
  cout << "--t = " << endl << t << endl;
  // cout << "--R_inv = " << endl << R.t() << endl;
  // cout << "--t_inv = " << endl << -R.t() * t << endl;

  cout << "[Bundle Adjustment]"  << endl;


  return 0;
}


void find_feature_matches(const Mat& src1,
                          const Mat& src2, 
                          vector<KeyPoint>& keypoints_1,
                          vector<KeyPoint>& keypoints_2,
                          vector<DMatch>& matches)
{
  // 1. 特征提取
  int num_keypints = 400;
  Ptr<ORB> orb = ORB::create(num_keypints);
  orb->detect(src1, keypoints_1);
  orb->detect(src2, keypoints_2);

  // 2. 提取描述子
  Mat description1, description2;
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  descriptor->compute(src1, keypoints_1, description1);
  descriptor->compute(src2, keypoints_2, description2);

  // 3.进行匹配
  vector<DMatch> matches_;
  BFMatcher matcher(NORM_HAMMING);
  matcher.match(description1, description2, matches_);

  // 4. 寻找最小最大值
  double max_dist = 0, min_dist = 1000;
  for(int i = 0; i < matches_.size(); i++)
  {
    double dist = matches_[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }
  cout << "[match result] --Total KeyPoint has: " << matches_.size() << endl;

  // 5. 筛选
  for(int i = 0; i < matches_.size(); i++)
  {
    if(matches_[i].distance < max_dist/2)
      matches.push_back(matches_[i]);
  }
  cout << "[match result] --after Num of KeyPoint has : " << matches.size() << endl;

  // 6. 画图显示一下
  Mat res;
  cout << "[draw] --start drawing..." << endl;
  drawMatches(src1, keypoints_1, src2, keypoints_2, matches, res);
  imshow("match result", res);
  waitKey();
  cout << "[draw] --draw end!" << endl;
}


Point2d pixel2cam(const Point2d& p, const Mat& K)
{
  return Point2f( (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
                  (p.y - K.at<double>(1,2)) / K.at<double>(1,1));
}

void pose_estimation_3d3d(const vector<Point3f>& pts1, 
                          const vector<Point3f>& pts2, 
                          Mat &R, Mat &t)
{
  // 1. 计算平均值
  Point3f p1, p2;
  double N = pts1.size();
  for(int i = 0; i < N; i++)
  {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 = Point3f(Vec3f(p1) / N);
  p2 = Point3f(Vec3f(p2) / N);
  cout << "--p1 = " << p1 << endl;
  cout << "--p2 = " << p2 << endl;
  // 2. 移除平均值
  vector<Point3f> q1(N), q2(N);
  for(int i = 0; i < N; i++)
  {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // 计算 q1*q2*T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for(int i = 0; i < N; i++)
  {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  cout << "W = " << W << endl;

  // SVD
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  cout << "U = " << U <<  endl;
  cout << "V = " << V <<  endl;

  Eigen::Matrix3d R_ = U*(V.transpose());
  if(R_.determinant() < 0) {
    R_ = -R_;
  }
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

  R = (Mat_<double>(3,3) << 
        R_(0,0), R_(0,1), R_(0,2),
        R_(1,0), R_(1,1), R_(1,2),
        R_(2,0), R_(2,1), R_(2,2)
        );
  t = (Mat_<double>(3,1) <<
        t_(0,0),t_(0,1), t_(0,2)
          );
}
