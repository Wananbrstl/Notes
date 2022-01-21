#include<iostream>
#include<math.h>
#include "Eigen/Core"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    // B
    // 创建矩阵
    Eigen::Matrix<double, 3, 3> rot;
    // 矩阵的运算
    rot << 1, 2, 3,
           4, 5, 6, 
           7, 8, 9 ;
    cout << "matrix :" << endl << rot << endl;
    cout << "size = " << rot.size() << endl;
    cout << "row() =  " << rot.row(1) << endl;
    cout << "rows = " << rot.rows() << endl;
    cout << "col() = " << rot.col(1) << endl;
    cout << "cols() = " << rot.cols() << endl;
    cout << "rot(0) = " << rot(0) << endl;
    cout << "rot(1,1) = " << rot(1,1) << endl;

    cout << rot.sum() << endl;


    cout << "========= 矩阵快运算" << endl;
    Matrix2d res = rot.block<2,2>(0,0);
    cout << res <<endl;


    cout << "======== 点和☕运算" << endl;
    Vector3d v1;
    v1<< 1,2,3;
    Vector3d v2;
    v2<< 2,3,4;
    double x = v1.dot(v2);
    
    cout << " dot -- " << x  << endl;
    cout << " cross - -" << (v1.cross(v2))<< endl;


    // rot.resize(8);
    // cout << "rot.resize(8) = "  << rot << endl;
    // rot.setRandom(3,3);
    // cout << "rot.setRandom(3, 3)" << rot << endl;
    // Eigen::Matrix3d rot2;
    // rot2.setZero() ;
    
    // cout << "矩阵分块" << endl;
    // // cout << "rot.head(3)  = " << rot.head(1) << endl;
    // Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
    // cout << "mat = " << endl << mat << endl;
    // cout << "mat(rot)" << mat.eigenvalues() << endl;

    return 0;
}
