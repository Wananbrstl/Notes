#include <iostream>
#include <cmath>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/so3.h"
#include "sophus/se3.h"

using namespace std;

int main(int argc, char *argv[])
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)).toRotationMatrix();
    // 通过矩阵构造李群SO3
    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_V(0,0,M_PI/2);
    // 构造一个四元素
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);
    cout << "SO(3) from matrix: " << SO3_R << endl;
    cout << "SO(3) from vector: " << SO3_V << endl;
    cout << "SO(3) from quaternion : " << SO3_q << endl;
    
    cout << "李群的本质就是一个矩阵， 李代数的本质是一个向量" << endl;
    cout << "李群 --log--> 李代数" << endl;
    cout << "李代数 --exp--> 李群" << endl;
    Eigen::Vector3d so33(1,1,1);
    Sophus::SO3 SO3 = Sophus::SO3::exp(so33);
    Eigen::Vector3d lie3 = Sophus::SO3::log(SO3);
    cout << "lie3 = " << lie3.transpose() << endl;
    cout << "SO3 = " << SO3 << endl;
    cout << "lie.hat() = " << Sophus::SO3::hat(lie3) << endl;
    cout << "SO3.vee() = " << Sophus::SO3::vee( Sophus::SO3::hat(lie3)).transpose() << endl;


    // 增量扰动模型
    Eigen::Vector3d update_so3(1e-4,0,0);
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
    cout << "SO3 update = " <<SO3_updated << endl;
    cout << "log运算将李群转化成李代数。\n hat运算将向量转成反对称阵 \n vee运算将反对称阵转成向量。" << endl;

    // 对SE(3) 进行操作
    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3 SE3_Rt(R,t);
    Sophus::SE3 SE3_qt(q,t);
    cout << "SE3 from R, t = " << SE3_Rt << endl;
    cout << "SE3 from q, t = " << SE3_qt << endl;
    typedef Eigen::Matrix<double, 6 , 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 hat = " << endl << Sophus::SE3::hat(se3) << endl;
    cout << "se3 vee = " << endl << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;
    cout << "李代数se(3)说白了就是一个6维向量" << endl;
    cout << "李群SE(3)说白了就是一个4x4的矩阵" << endl;
    // 扰动模型
    cout << "扰动模型" << endl;
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_update = Sophus::SE3::exp(update_se3)*SE3_Rt;
    cout << "SE3 update = " << endl << SE3_update.matrix() << endl;
    return 0;
}
