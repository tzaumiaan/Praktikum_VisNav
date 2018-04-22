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

Matrix4d q_op_plus(Vector4d v, bool is_plus)
{
  Matrix4d m;
  Vector3d imag = v.head(3);
  double real = v(3);
  Matrix3d imag_skew_sym = skew_sym_matrix(imag);
  m.topLeftCorner(3,3) = (is_plus)? imag_skew_sym: -imag_skew_sym;
  m.bottomLeftCorner(1,3) = - imag.transpose();
  m.topRightCorner(3,1) = imag;
  m += real * Matrix4d::Identity();
  return m;
}

int main()
{
  double angle;
  Vector3d axis;
  cout.precision(4);
  
  
  cout << "Task 1" << endl; 
  // assign a random angle and axis
  angle = M_PI/2;
  axis << 1.f/3.f, -2.f/3.f, 2.f/3.f;
  cout << "Given rotation vector with angle = " << angle;
  cout << " and rotation axis = (" << axis.transpose() << ")^T" << endl;
  cout << "Rotation matrix of Rodrigues' =" << endl
       << rotation_matrix_rodrigues(angle, axis) << endl;
  cout << "Rotation matrix from library =" << endl
       << rotation_matrix_library(angle, axis) << endl;
  cout << endl;
  
  cout << "Task 2" << endl; 
  // initialize q1 and q2 as random unit quaternions
  Quaterniond q1 = Quaterniond::UnitRandom();
  Quaterniond q2 = Quaterniond::UnitRandom();
  // coeffs is presented as (x,y,z,w), with imaginary part first and real part last
  cout << "q1 = (" << q1.coeffs().transpose() << ")^T" << endl;
  cout << "q2 = (" << q2.coeffs().transpose() << ")^T" << endl;
  Quaterniond q3 = q1*q2;
  Vector4d v4 = q_op_plus(q1.coeffs(), true) * q2.coeffs();
  Vector4d v5 = q_op_plus(q2.coeffs(), false) * q1.coeffs();
  cout << "q1 q2     = (" << q3.coeffs().transpose() << ")^T" << endl;
  cout << "q1^ +  q2 = (" << v4.transpose() << ")^T" << endl;
  cout << "q2^(+) q1 = (" << v5.transpose() << ")^T" << endl;
  
  // conversion between quaternion and rotation matrix
  Quaterniond q = q1;
  cout << "Original q = (" << q.coeffs().transpose() << ")^T" << endl;
  Matrix3d rotation_matrix = q.toRotationMatrix();
  cout << "Converted rotation matrix R =" << endl
       << rotation_matrix << endl;
  q = Quaterniond(rotation_matrix);
  cout << "Now convert back to quaternion q' = (" 
       << q.coeffs().transpose() << ")^T" << endl;
  cout << endl;

  cout << "Task 3" << endl; 
  q1 = Quaterniond(0.55, 0.3, 0.2, 0.2).normalized();
  q2 = Quaterniond(-0.1, 0.3, -0.7, 0.2).normalized();
  Vector3d t1 (0.7, 1.1, 0.2);
  Vector3d t2 (-0.1, 0.4, 0.8);
  Vector3d p1 (0.5, -0.1, 0.2);
  
  cout << "Object in R1's coord and R1's pos = (" << p1.transpose() << ")^T" << endl;
  
  // inversion of q1
  Quaterniond q1_inv = q1.conjugate().normalized();
  // cout << q1.coeffs() << endl << q1_inv.coeffs() << endl;
  // cout << (q1 * q1_inv).coeffs() << endl;
  
  // rotate back to world coordinates
  Vector3d p1o = q1_inv * p1; // overloaded *, actually q1_inv dot p1 dot q1 in math
  cout << "Object in world coord and R1's pos = (" << p1o.transpose() << ")^T" << endl;
  //cout << q1*p << endl;

  // translate to R2's position
  Vector3d p2o = p1o + t1 - t2;
  cout << "Object in world coord and R2's pos = (" << p2o.transpose() << ")^T" << endl;
  
  // rotate to R2's coordinates
  Vector3d p2 = q2 * p2o;
  cout << "Object in R2's coord and R2's pos = (" << p2.transpose() << ")^T" << endl;

  cout << endl;

  return 0;
}
