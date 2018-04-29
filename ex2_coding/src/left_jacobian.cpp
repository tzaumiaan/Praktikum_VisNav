#include <iostream>
#include <cmath>
using namespace std; 

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

#include <sophus/se3.hpp>

typedef Matrix<double,6,1> Vector6d;

Matrix3d skew_sym_matrix(Vector3d vec)
{
  Matrix3d m;
  m << 0, -1*vec(2), vec(1), vec(2), 0, -1*vec(0), -1*vec(1), vec(0), 0;
  return m;
}

Matrix3d rotation_matrix_library(double angle, Vector3d axis)
{
  AngleAxisd v(angle, axis);
  return v.matrix();
}

Matrix3d rotation_matrix_rodrigues(double angle, Vector3d axis)
{
  double c = cos(angle);
  double s = sin(angle);
  Matrix3d m = c * Matrix3d::Identity();
  m += (1-c) * (axis * axis.transpose());
  m += s * skew_sym_matrix(axis); 
  return m;
}

int main( int argc, char** argv )
{
    // given a translation vector
    Vector3d rho(1,0,0);
    cout << "given translation vector rho = (" << rho.transpose() << ")^T" << endl;
    // given a rotation matrix
    double theta = M_PI/3.f;
    Vector3d a(0,0,1);
    Matrix3d phi = AngleAxisd(theta, a).toRotationMatrix();
    cout << "given rotation matrix phi = " << endl << phi << endl;
    cout << "from theta = " << theta 
         << " and axis = (" << a.transpose() << ")^T" << endl;
    // use rotation matrix and translation vector to initialize SE3
    Sophus::SE3d xi(phi, rho);
    cout << "SE3 xi = " << endl << xi.matrix() << endl;
    // Lie algebra of xi
    Vector6d xi_se3 = xi.log();
    cout << "se3 of xi = (" << xi_se3.transpose() << ")^T" << endl;
    // exp map of Lie algebra
    Sophus::SE3d xi_se3_exp = Sophus::SE3d::exp(xi_se3);
    cout << "its rotation matrix from exponent map = " << endl
         << xi_se3_exp.rotationMatrix() << endl;
    cout << "rotation matrix of Rodrigues' =" << endl
         << rotation_matrix_rodrigues(theta, a) << endl;
    return 0;
}
