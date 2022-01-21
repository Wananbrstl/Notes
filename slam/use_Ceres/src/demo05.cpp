/*
 * BA优化
 *
 */
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <Sophus/core>
#include "matplotlibcpp.h"

using namespace std;

// 构建残差
class BAGNCostFunctor{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      BAGNCostFunctor(Eigen::Vector2d observed_p, Eigen::Vector3d observed_P):
                      observed_p_(observed_p), observed_P_(observed_P){}

    virtual ~BAGNCostFunctor(){}

    // 参数 残差 雅可比
    virtual bool Evaluate(double const * const param,
                          double* reiduals,
                          double **jacobians)
    {
      Eigen::Map<const Eigen::Matrix<double,6,1>> T_se3(*param);
      Sophus::SE3 T_SE3 = Sophus::SE3::exp(T_se3);
      Eigen::Vector3d Pc = T_SE3 * observed_P_;
      Eigen::Matrix3d K;
      double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
      K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

      // 计算残差
      Eigen::Vector2d reidual = observed_p_ - (K*Pc).hnormalized();
      residuals[0] = residual[0];
      residuals[1] = residual[1];

      if(jacobians != NULL){
        if(jacobians[0]!=NULL){
          // 2x6的雅可比矩阵
          Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobians[0]);
          double x = Pc[0];
          double y = Pc[1];
          double z = Pc[2];
      
          double x2 = x*x;
          double y2 = y*y;
          double z2 = z*z;

          J(0,0) = fx/z;
          J(0,1) = 0;
          J(0,2) = -fx*x/z2;
          J(0,3) = -fx*x*y/z2;
          J(0,4) = fx+fx*x2/z2;
          J(0,5) = -fx*y/z;
          
          J(1,0) = 0;
          J(1,1) = fy/z;
          J(1,2) = -fy*y/z2;
          J(1,3) = -fy-fy*y2/z2;
          J(1,4) = fy*x*y/z2;
          J(1,5) = fy*x/z;
        }
      }
      return true;
    }

  private:
    const Eigen::Vector2d observed_p_;
    const Eigen::Vector3d observed_P_;
};

int main(int argc, char const* argv[])
{
  // 首先构造问题
  Sophus::Vector6d se3;

  ceres::Problem problem;
  for(int i = 0; i < n_; i++){
    ceres::CostFunction* costfunc = new BAGNCostFunctor(p2d[i], p3d[i]);
    problem.AddResidualBlock(costfunc, NULL, se3.data());
  }

  ceres::Solver::Options options;
  options.dynamic_sparsity = true;
  options.max_num_iterations = 100;
  options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
  options.minizer_type = ceres::TRUST_REGION;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.minimizer_progress_to_stdout = true;
  options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  ceres::Solver::Summary summary;
  ceres::Solve(problem, &options, &summary);
  cout << summary.BriefReport() << endl;
  cout << "[done] --estimated pose: \n" << Sophus::SE3::exp(se3).matrix() << endl;
  
  return 0;
}
