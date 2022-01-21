/**
 * @file test_linearSolve.cpp
 * @brief This file is for test the Eigen linear Solve
 * @author wananbrstl
 * @version 1.0
 * @date 2022-01-02
 */

#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

int main()
{
    Eigen::Matrix3d A;
    Vector3d b;
    A << 1,2,3, 4,5,6, 7,8,9;
    b << 3,3,4;
    cout << "A = " << A << endl;
    cout << "b = " << b << endl;
    Matrix3d q = A.colPivHouseholderQr().matrixQ();
    Matrix3d r = A.colPivHouseholderQr().matrixR();
    cout << "Q = " << endl << q << endl;
    cout << "R = " << endl << r << endl;
    return 0;
}
