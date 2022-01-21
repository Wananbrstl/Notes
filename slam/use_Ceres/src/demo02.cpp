/*
优化求解过程
1. 首先，需要自己构建一个代价函数
2. 其次，给定初值  
*/
#include <iostream>
#include "ceres/ceres.h"
using namespace std;
using namespace ceres;

struct CostFuntor
{
    template<class T>
    bool operator()(const T* const x, T* residual) const 
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};


int main(int argc, char  *argv[])
{
    double initial_x = 10;
    double x = initial_x;

    // 3. 创建一个问题对象
    Problem problem;

    // 4. 构建一个代价函数
    CostFunction* const_function = new AutoDiffCostFunction<CostFuntor, 1, 1>(new CostFuntor);
    problem.AddResidualBlock(const_function, nullptr, &x);
    
    // 5. 创建一个求解器，并且求解
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem,&summary);

    // 6. 显示结果
    cout << summary.BriefReport() << "\n";
    cout << "x = " << initial_x << "->" <<  x << endl;
    return 0;
}
