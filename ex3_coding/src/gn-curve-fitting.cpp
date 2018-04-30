//
// Created by Xiang on 2017/12/15.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

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

    // start gauss-newton
    int iterations = 100;    // number of iterations
    double cost = 0, lastCost = 0;  // total cost and last total cost

    for (int iter = 0; iter < iterations; iter++) {

        Matrix3d H = Matrix3d::Zero();             // NOTE Hessian = J^T J in Gauss-Newton
        Vector3d b = Vector3d::Zero();             // bias
        cost = 0;

        for (int i = 0; i < N; i++) {
            double xi = x_data[i], yi = y_data[i];  // the i-th data
            // start your code here
            double error = 0;   // compute the error of i-th data
            Vector3d J; // compute Jacobian
            J[0] = 0;  // de/da
            J[1] = 0;  // de/db
            J[2] = 0;  // de/dc

            H += J * J.transpose(); // set Hessian
            b += -error * J;    // set bias
            // end your code here

            cost += error * error;
        }

        // solve normal equation Hx=b
        // start your code here
        Vector3d dx;    // update
        // end your code here

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost > lastCost) {
            // cost increased, break the iteration
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update estimation
        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        cout << "total cost: " << cost << endl;
    }

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}