#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "matplotlibcpp.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
    // 1. 产生数据
    cv::RNG rng;
    double w_sigma = 1.0;
    int N = 100;
    double a = 1, b = 2, c = 3;
    double a_ = 2, b_ = 3, c_ = 4;
    // TODO 注意Vector的初始化
    // vector初始化会导致初始化那么多零;
    // std::vector<double> x_data(N), y_data(N);
    std::vector<double> x_data, y_data;
    for(int i = 0; i < N; i++) 
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a*x*x + b*x + c) + rng.gaussian(w_sigma));
    }
    for(auto i: x_data)
        cout << i << endl;
    cout << "x_data.size" << x_data.size() << endl;
    plt::plot(x_data, y_data, "r.");
    plt::show();

    // 2. 进行求解 Gauss-Newton法
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    // TODO 高斯牛顿法
    // 迭代信息
    int num_iter = 100;
    double cost = 0, lastCost = 0;
    for(int it = 0; it < num_iter; it++)
    {
        // 每一次迭代求解
        cost = 0;
        Matrix3d H = Matrix3d::Zero();
        Vector3d B = Vector3d::Zero();
        for(int i = 0; i < N; i++)
        {
            double xi = x_data[i], yi = y_data[i];
            double error = yi - exp(a_ *xi*xi + b_ *xi + c_);
            // Jacobian
            Vector3d J;
            J[0] = -xi * xi * exp(a_*xi*xi + b_*xi + c_);
            J[1] = -xi * exp(a_*xi*xi + b_*xi + c_);
            J[2] = - exp(a_*xi*xi + b_*xi + c_);
            H    += J * J.transpose();
            B    += - error * J;
            cost += error * error;
        }
        
        Vector3d dx = H.ldlt().solve(B);
        // 循环结束条件
        if(isnan(dx[0]))
        {
            cout << "result is nan" << endl;
            break;
        }
        if(it > 0 && cost > lastCost)
        {
            cout << "cost > lastCost" << endl;
            break;
        }
        
        // 更新
        a_ += dx[0];
        b_ += dx[1];
        c_ += dx[2];

        lastCost = cost;
        // cout << cost <<"\t" << dx.transpose() << "\t" << a_ <<" " << b_ << " " << c_ << endl;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds." << endl;
    cout << "estimated abc = " << a_ << ", " << b_ << " ," << c_  << endl;
    return 0;
}

