//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <sophus/se3.hpp>

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../data/p3d.txt";
string p2d_file = "../data/p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    double x, y, z;
    ifstream f_p3d(p3d_file);
    while(f_p3d >> x >> y >> z)
    {
        p3d.push_back(Vector3d(x, y, z));
    }
    ifstream f_p2d(p2d_file);
    while(f_p2d >> x >> y)
    {
        p2d.push_back(Vector2d(x, y));
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            Vector2d e(0,0);    // error
            // START YOUR CODE HERE 
            Matrix3d T_R = T_esti.rotationMatrix();
            Vector3d T_t = T_esti.translation();
            Vector3d TP =  T_R*p3d[i] + T_t; // known as (X' Y' Z')^T
            Vector3d KTP = K*TP;
            e = p2d[i] - KTP.head(2)/KTP(2);
            cost += e.squaredNorm();
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            double X = TP(0), Y = TP(1), Z = TP(2);
            J << -fx/Z, 0, fx*X/(Z*Z), fx*X*Y/(Z*Z), -fx - fx*X*X/(Z*Z), fx*Y/Z,
                 0, -fy/Z, fy*Y/(Z*Z), fy + fy*Y*Y/(Z*Z), -fy*X*Y/(Z*Z), -fy*X/Z;
            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.inverse() * b;
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = T_esti * Sophus::SE3d::exp(dx);
        //cout << "debug\n" <<T_esti.matrix()<<endl;
        // END YOUR CODE HERE

        lastCost = cost;

        cout << "iteration " << iter << " cost=" /*<< cout.precision(12)*/ << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
