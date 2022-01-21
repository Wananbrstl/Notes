#include<iostream>
#include<cmath>
using namespace std;

#include"sophus/so3.h"
#include"sophus/se3.h"
#include<Eigen/Core>
#include<Eigen/Geometry>

int main(int argc, char** argv)
{
  // Eigen   库是一个提供矩阵运算的库
  // Souphus 库是一个李代数和李群的库
  // Souphus 中 李群和李代数的创建可以使用Eigen或者一个Vector来创建, 反正代表的是旋转、平移就行
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
  Eigen::Vector3d rotate(0,0,1);
  cout << "--rotate vector : " << rotate.transpose() << endl;
  Sophus::SO3 SO3_R(R);
  Sophus::SO3 SO3_rotate(rotate(0,0), rotate(1,0), rotate(2,0));
  Sophus::SO3 SO3_v(0, 0, M_PI/2);
  Eigen::Quaterniond q(R);
  Sophus::SO3 SO3_q(q);
  cout << "--SO(3) from matix = " << SO3_R << endl;
  cout << "--SO(3) from vector = " << SO3_v << endl;
  cout << "--SO(3) from Quaterniond = " << SO3_q << endl;
  cout << "--so(3) from vector = " << SO3_rotate << endl;


  // 知道了李群， 求出其李代数 
  // 说白了 就是对数和指数的关系
  Eigen::Vector3d so3 = SO3_R.log();
  cout << "--so(3) = " << so3 << endl;
  // hat 就是其泛对称矩阵
  Eigen::Matrix3d so3_hat = Sophus::SO3::hat(so3);
  cout << "--so3_hat = " << endl << so3_hat << endl;
  // 也可以进行反变换
  Eigen::Vector3d so3_vee = Sophus::SO3::vee(so3_hat);
  cout << "--so3_hat vee = " << so3_vee.transpose() << endl;

  // SE（3）的操作大同小异
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3 SE3_R(R, t);
  Sophus::SE3 SE3_q(q, t);
  cout << "--SE(3) form R and t : " << SE3_R  << endl;
  cout << "--SE(3) form q and t : " << SE3_q  << endl;

  // 转换
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_R.log();
  cout << "--se(3) is " << se3 << endl;

  // 同样也有vee 和 hat 操作
  Eigen::Matrix4d se3_hat = Sophus::SE3::hat(se3);
  Eigen::Matrix<double, 6,  1> se3_vee = Sophus::SE3::vee(se3_hat);
  cout << "--SE(3) hat is :  " << endl << se3_hat << endl;
  cout << "--SE(3)'hat vee is : " << se3_vee.transpose() << endl; 


  // 最后演示扰动模型
  Vector6d updata_se3;
  updata_se3.setZero();
  updata_se3(0,0) = 1e-4d;
  Sophus::SE3 SE3_updatad = Sophus::SE3::exp(updata_se3)*SE3_R;
  cout << "--SE3 updatad" << endl << SE3_updatad.matrix() << endl;

  return 0;
}
