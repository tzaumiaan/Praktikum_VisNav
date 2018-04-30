// Created by xiang on 12/21/17.
#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

// curve fitting vertex, a Vector3d (abc)
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        // reset to zero
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double *update) override {
        // update
        _estimate += Eigen::Vector3d(update);
    }

    // don't need read and write functions
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

// curve fitting edge, unary edge
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    virtual void computeError() override {
        // TODO implement the error computation
        /// start your code here

        /// end your code here
    }

    virtual void linearizeOplus() override {
        // TODO implement jacobian computation here
        /// start your code here
        _jacobianOplusXi = Eigen::Vector3d(0, 0, 0);
        /// end your code here
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

public:
    double _x;  // data, y is in the _measurement
};

int main(int argc, char **argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;         // ground-truth values
    double ae = 2.0, be = -1.0, ce = 5.0;        // estimated
    int N = 100;                                 // num of data
    double w_sigma = 1.0;                        // noise sigma
    cv::RNG rng;                                 // OpenCV random variable generator

    vector<double> x_data, y_data;      // observation data
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma));
    }

    // setup g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;  // use 3x1 block solver
    std::unique_ptr<Block::LinearSolverType> linearSolver(
            new g2o::LinearSolverDense<Block::PoseMatrixType>()); // linear solver

    // use levernberg-marquardt here (or you can choose gauss-newton)
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<Block>(std::move(linearSolver)));
    g2o::SparseOptimizer optimizer;     // graph optimizer
    optimizer.setAlgorithm(solver);   // solver
    optimizer.setVerbose(true);       // open the output

    // TODO build the optimization problem
    /// start your code here

    /// end your code here

    // call optimization
    cout << "start optimization" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    // output the estimated value
    cout << "estimated a,b,c = " << ae << ", " << be << ", " << ce << endl;

    return 0;
}