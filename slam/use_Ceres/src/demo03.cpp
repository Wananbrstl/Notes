/*
 *  本例程用于求解问题 $ min 1/2 * (x - 10)^2 $问题
 */
#include <iostream>
#include <ceres/ceres.h>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std;

// 首先构建残差函数(使用方函数)
struct CostFunctor{
  template<class T>
    bool operator()(const T* const x, T* residual ) const {
      residual[0] = 10.0 - x[0];
      return true;
    }
};

int main(int argc, char const* argv[])
{
  google::InitGoogleLogging(argv[0]);
  // 首先
  double ini_x = 5.0;
  double x = ini_x;
  ceres::Problem problem;
  ceres::CostFunction* costfunc = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(costfunc, NULL, &x);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);

  cout << summary.BriefReport() << endl;
  cout << "-- x = " << x << endl;
  return 0;
}
/*
#include<iostream>
#include<ceres/ceres.h>

using namespace std;
using namespace ceres;

//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

//主函数
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // 寻优参数x的初始值，为5
  double initial_x = 5.0;
  double x = initial_x;

  // 第二部分：构建寻优问题
  Problem problem;
  //使用自动求导，将之前的代价函数结构体传入，第一个1是输出维度，即残差的维度，第二个1是输入维度，即待寻优参数x的维度。
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  //向问题中添加误差项，本问题比较简单，添加一个就行。
  problem.AddResidualBlock(cost_function, NULL, &x);

  //第三部分： 配置并运行求解器
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR; //配置增量方程的解法
  options.minimizer_progress_to_stdout = true;//输出到cout
  Solver::Summary summary;//优化信息
  Solve(options, &problem, &summary);//求解!!!

  std::cout << summary.BriefReport() << "\n";//输出优化的简要信息
  //最终结果
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  return 0;
}
*/
