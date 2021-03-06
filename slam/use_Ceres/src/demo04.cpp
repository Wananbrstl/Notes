/*
 * 本例程用来拟合曲线 $ e^(3x^2 + 2x + 1) $
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

// 定义损失函数
struct Cost{
  Cost(double x, double y): _x(x), _y(y){}

  template<class T>
  bool operator()(const T* const abc, T* rediual) const
  {
    rediual[0] = _y - ceres::exp(abc[0]*_x*_x + abc[1]*_x + abc[2] );
    return true;
  }
  private:
    double _x, _y;
};

int main(int argc, char const* argv[])
{
  // 1. 产生数据
  double a_ = 3, b_ = 2, c_ = 1;
  double a = 1, b = 0, c = 3;
  double w_sigma = 1;
  int N = 100;
  cv::RNG rng;
  vector<double> x_data, y_data;
  // [0,1]
  for(int i = 0; i < N; i++)
  {
    double x = (double)i/N;
    x_data.push_back(x);
    y_data.push_back(exp(a_*x*x + b_*x + c_) + rng.gaussian(w_sigma));
  }
  // 2. 构建问题
  double abc[3] = {a, b, c};
  ceres::Problem problem;
  for(int i = 0; i < N; i++)
  {
    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<Cost, 1, 3>(new Cost(x_data[i], y_data[i])),
                            nullptr,
                            abc);
  }
  // 3. 构建求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  
  // 4. 输出
  ceres::Solver::Summary summary;
  // 5. 记录时间
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

  // 6. 打印
  cout << summary.BriefReport() << endl;
  cout << "[done] --use time : " << time_used.count() <<"second." << endl;
  cout << "[result] --a = " << abc[0] << endl;
  cout << "[result] --b = " << abc[1] << endl;
  cout << "[result] --c = " << abc[2] << endl;
  cout << "[figure] loading....." << endl;

  // 7. 输出图像
  vector<double> y_data_;
  for(int i = 0; i < N; i++)
  {
    double x = (double)i / N;
    double y = exp(abc[0]*x*x + abc[1]*x + abc[2]);
    y_data_.push_back(y);
  }
  plt::plot(x_data, y_data, "r.");
  plt::plot(x_data, y_data_, "b");
  plt::title("curve fitting  figure");
  plt::xlabel("x");
  plt::ylabel("y");
  plt::show();

  cout << "[figure] result." << endl;
  return 0;
}
