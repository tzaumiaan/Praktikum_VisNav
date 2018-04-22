#include <iostream>
using namespace std;

#include <Eigen/Geometry>
using namespace Eigen;

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

int main()
{
  double angle;
  Vector3d axis;
  cout.precision(3);
  
  angle = M_PI/6;
  axis << 1, -2, 1;
  
  cout << "Given rotation vector with angle = " << angle;
  cout << " and rotation axis = (" << axis.transpose() << ")^T" << endl;
  cout << "Rotation matrix of Rodrigues' =" << endl;
  cout << rotation_matrix_rodrigues(angle, axis) << endl;
  cout << "Rotation matrix from library =" << endl;
  cout << rotation_matrix_library(angle, axis) << endl;

}
