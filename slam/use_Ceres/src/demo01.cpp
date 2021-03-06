#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;
//namespace plt = matplotlibcpp;

struct CURVE_FTTING_COST{
  CURVE_FTTING_COST(double x, double y) : _x(x), _y(y){}

  // 计算残差
  template<class T>
  bool operator()(const T* const abc, T* residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
    return true;
  }

  const double _x, _y;
};

int main(int argc, char const* argv[])
{
  double ar = 2.0, br = 3.0, cr = 2.0;
  double ae = 1.0, be = 2.0, ce = 1.0;
  int N = 100;
  double w_sigma = 1.0;
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;

  // 生成拟合数据
  vector<double> x_data, y_data;
  for(int i = 0; i < N; i++)
  {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }
  double abc[3] = {ae, be, ce};

  // 构建优化
  ceres::Problem problem;
  for(int i = 0; i < N; i++)
  {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CURVE_FTTING_COST, 1, 3>(new CURVE_FTTING_COST(x_data[i], y_data[i])),
                             nullptr,
                             abc);
  }
  // 配置求解器
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "[done] solve time cost = " << time_used.count() << "second" << endl;
  
  cout << summary.BriefReport() << endl;
  cout << "estimated a, b, c = ";
  for(auto a:abc) cout << a << "  " ;

  /*
  plt::plot(x_data, y_data);
  plt::pause(2);
  plt::show();
  */
  return 0;
}
