#include <iostream>
#include <cmath>
using namespace std;

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "sophus/so3.h"
#include "sophus/se3.h"

int main(int argc, char const *argv[])
{
    // 通过旋转矩阵构造
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_v(0, 0, M_PI/2);
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);

    cout << "SO(3) from matrix \t:" << SO3_R << endl;
    cout << "SO(3) from vector \t:" << SO3_v << endl;
    cout << "SO(3) from quaternion \t:" << SO3_q << endl;

    // 通过对数映射得到李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // hat 为向量到反对陈矩阵
    cout << "so3 hat = " << Sophus::SO3::hat(so3) << endl;
    // 相对地 vee为泛对称矩阵到向量
    cout << "so3 vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3 SO3_update = Sophus::SO3::exp(update_so3)*SO3_R;
    Sophus::SO3 SO3_update_ = Sophus::SO3::exp(update_so3 + Eigen::Vector3d(0,0,M_PI/2) );
    cout << "so3 update = " << SO3_update << endl;
    cout << "so3 updata2 =  " << SO3_update_ << endl;
    cout << "=========================" << endl;
    // 对SE3来说大同小异
    Eigen::Vector3d t(1,0,0);
    Sophus::SE3 SE_Rt(R,t);
    Sophus::SE3 SE_Qt(q,t);
    cout << "se3 from R,t = " << endl << SE_Rt << endl;
    cout << "se3 from q,t = " << endl << SE_Qt << endl;
    // 李代数se(3)是一个刘为向量
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    return 0;
}
