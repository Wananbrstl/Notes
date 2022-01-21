#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>

using namespace std;
using namespace cv;


void find_feature(const Mat& image1, 
                  const Mat& image2,
                  vector<KeyPoint>& keypoints_1,
                  vector<KeyPoint>& keypoints_2,
                  vector<DMatch>& matches)
{
  cout << "开始查找特征点" <<endl;
  // 1. 首先就是进行特征点的查找
  int num_feature = 500;
  Ptr<ORB> orb = ORB::create(num_feature);
  orb->detect(image1, keypoints_1);
  orb->detect(image2, keypoints_2);
  
  // 2. 描述子的描述
  Mat descriptor1, descriptor2;
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  descriptor->compute(image1, keypoints_1, descriptor1);
  descriptor->compute(image2, keypoints_2, descriptor2);
  
  // 3. 特征值匹配
  vector<DMatch> match;
  //BFMatcher matcher(NORM_HAMMING);
  //matcher.match(descriptor1, descriptor2, match);
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  matcher->match(descriptor1, descriptor2, match);

  // 4.筛选
  double min_dist = 10000, max_dist = 0;
  for(int i = 0; i < descriptor1.rows; i++)
  {
    double dist = match[i].distance;
    if(dist < min_dist) min_dist = dist;
    if(dist > max_dist) max_dist = dist;
  }

  printf("--the max dist is: %.2f \n", max_dist);
  printf("--the min dist is: %.2f \n", min_dist);
  
  //选出合格的来
  for(int i = 0; i < match.size();i++)
  {
    if(match[i].distance <= max(2*min_dist, 30.0) )
    {
      matches.push_back(match[i]);
      cout << match[i].queryIdx << endl;
    }
  }

  // 画图看看
  Mat res;
  drawMatches(image1, keypoints_1, image2, keypoints_2, matches, res);
  printf("--Total has %d KeyPoints \n", (int)matches.size());
  imshow("匹配", res);
  waitKey();
}

// 2d2d estimation
void pose_estimation_2d2d(const vector<KeyPoint>& keypoints_1,
                          const vector<KeyPoint>& keypoints_2,
                          const vector<DMatch> matches,
                          Mat& R, Mat& t)
{
  // 1. 内参矩阵
  Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 
                                0, 521.0, 249.7,
                                0, 0, 1);
  
  // 2. 找出特征点point
  vector<Point2d> points1, points2;
  for(int i = 0; i < (int)matches.size(); i++)
  {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }
  cout << "--points1.size() = " << points1.size() << endl;

  // 3. 计算基础矩阵
  Mat fundamental_matrix;
  for(int i = 0; i < points1.size(); i++)
  {
    cout << "p1 = [" << points1[i].x << "," << points1[i].y<<"]" << endl;
    cout << "p2 = [" << points2[i].x << "," << points2[i].y<<"]" << endl;
  }
  fundamental_matrix = cv::findFundamentalMat(points1, points2, FM_8POINT);
  cout << "--Fundamental matrix is : "<< endl  << fundamental_matrix << endl;

  // 4. 本质矩阵
  Mat essential_matrix;
  Point2d principal_point(325.1, 249.7); // 光心
  double focal_length = 521; // 焦距
  essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
  cout << "--Essential Matrix is : " << endl << essential_matrix << endl;

  // 5.单应矩阵 意义不大
  Mat homography_matrix;
  homography_matrix = cv::findHomography(points1, points2, RANSAC, 3);
  cout << "--homography matrix is : " << endl << homography_matrix << endl;

  // 6. 恢复结构
  recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
  cout << "--R = " << endl << R << endl;
  cout << "--t = " << endl << t << endl;
}

Point2d pixel2cam(const Point2d& p, const Mat& K)
{
  return Point2d(
                 (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
                 (p.y - K.at<double>(1,2)) / K.at<double>(1,1)
                );
}


int main(int argc, char **argv)
{
  if(argc != 3) {
    cout << "请输入两个图片的位置" << endl;
    return 0;
  }
  // 1. 读取照片
  Mat src1 = imread(argv[1]);
  Mat src2 = imread(argv[2]);
  if(src1.empty() || src2.empty()) {
    cout << "请输入有效照片地址" << endl;
    return 0;
  }

  // 2. 找出特征点并且进行匹配
  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature(src1, src2, keypoints_1, keypoints_2, matches);

  // 3. 找出本质矩阵以及基础矩阵 
  Mat R, t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  // 4. 进行验证
  Mat t_x = (Mat_<double>(3,3) << 0, -t.at<double>(2,0), t.at<double>(1,0),
                                  t.at<double>(2,0), 0, -t.at<double>(0,0),
                                  -t.at<double>(1,0), t.at<double>(0,0), 0);
  cout << "--t^R = " << endl << t_x * R  << endl;

  // 5. 对积极和 x1 t^R x2 == 0
  Mat K = (Mat_<double>(3,3) << 520.9, 0, 325.1, 
                                0, 521.0, 249.7,
                                0, 0, 1);
  for(DMatch m: matches)
  {
    Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    Mat y1 = (Mat_<double>(3,1) << pt1.x, pt1.y, 1);
    Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    Mat y2 = (Mat_<double>(3,1) << pt2.x, pt2.y, 1);
    Mat d = y2.t() * t_x * R * y1;
    cout << "y2.t() = " <<  y2.t() << endl;
    cout << "y1  = " <<  y1 << endl;
    cout << "-- epopolar constraint = "  << d << endl;
  }
  return 0;
}


