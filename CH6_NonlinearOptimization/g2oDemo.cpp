#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <opencv2/core/core.hpp>
using namespace std;
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl() { _estimate << 0, 0, 0; } //估计值置0
  virtual void oplusImpl(const double *update) {
    _estimate += Eigen::Vector3d(update); //更新x, x_k+1=x_k+delta_x
  }
  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}
};

class CurveFittingEdge
    : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {

public:
  double _x;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}
  void computeError() {
    const CurveFittingVertex *v =
        static_cast<const CurveFittingVertex *>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement -
                   std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    //上面的=号写成了==号! 浪费了好多时间排错 -_- !!!
  }
  virtual bool read(std::istream &in){};
  virtual bool write(std::ostream &out) const {};
};

int main(int argc, char const *argv[]) {
  double a = 1.0, b = 2.0, c = 1.0;
  int N = 100;
  double w_sigma = 1.0; // noise
  cv::RNG rng;
  double abc[3] = {0};
  vector<double> x_data, y_data;
  cout << "Generating data ..." << endl;
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
    cout << x_data[i] << " " << y_data[i] << endl;
  }

  //构建图优化
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
  //矩阵块 每个误差项优化变量维度为3,误差值维度为1

  //新写法
  //   std::unique_ptr<Block::LinearSolverType> linearSolver(
  //       new g2o::LinearSolverDense<Block::PoseMatrixType>());
  //旧写法
  Block::LinearSolverType *linearSolver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();
  //线性方程求解器:稠密的增量方程

  //新写法
  //   std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));
  //旧写法
  Block *solver_ptr =
      new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
  // g2o更新了接口,书上的例子会报错
  // 旧写法参考:https://blog.csdn.net/weixin_41269344/article/details/82911047
  // 新写法参考:https://blog.csdn.net/weixin_38358435/article/details/79082733

  //梯度下降方法 从GN LM DOGLEG中选取
  //新写法
  //   g2o::OptimizationAlgorithmLevenberg *solver =
  //       new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  //旧写法
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(
          std::unique_ptr<Block>(solver_ptr));

  /*
    g2o::OptimizationAlgorithmGaussNewton *solver =
        new g2o::OptimizationAlgorithmGaussNewton( //高斯牛顿
            std::unique_ptr<Block>(solver_ptr));

    g2o::OptimizationAlgorithmDogleg *soler =
        new g2o::OptimizationAlgorithmDogleg //狗腿子法
        (std::unique_ptr<Block>(solver_ptr));
  */
  g2o::SparseOptimizer optimizer; //图模型
  optimizer.setAlgorithm(solver); //设置求解器
  optimizer.setVerbose(true);     //打开调试输出

  CurveFittingVertex *v = new CurveFittingVertex(); //向图中增加顶点
  v->setEstimate(Eigen::Vector3d(0, 0, 0));
  v->setId(0);
  optimizer.addVertex(v);

  for (int i = 0; i < N; i++) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y_data[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1.0 /
                         (w_sigma * w_sigma));
    optimizer.addEdge(edge);
  }

  //执行优化
  cout << "Start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(100);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "Solver time cost : " << time_used.count() << " seconds" << endl;

  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "Estimated model :" << abc_estimate.transpose() << endl;

  return 0;
}
