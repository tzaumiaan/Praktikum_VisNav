// Created by Xiang on 2017/12/15.
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// curve fitting cost
struct CURVE_FITTING_COST {
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    // TODO implement the error computation
    template<typename T>
    bool operator()(
            const T *const abc,     // abc
            T *residual) const     // residual
    {
        /// start your code here

        /// end your code here
        return true;
    }

    const double _x, _y;    // x,y data
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

    // construct the ceres problem
    ceres::Problem problem;
    // TODO build the optimization problem
    /// start your code here

    /// end your code here

    // setup solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;  // use QR to solve the normal equation
    options.minimizer_progress_to_stdout = true;   // output to cout

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    // call solve function
    ceres::Solve(options, &problem, &summary);

    // output the results
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = " << ae << ", " << be << ", " << ce << endl;

    return 0;
}

