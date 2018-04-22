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
  cout.precision(3);
  
  angle = M_PI/2;
  axis << 1.f/3.f, -2.f/3.f, 2.f/3.f;
  
  cout << "Given rotation vector with angle = " << angle;
  cout << " and rotation axis = (" << axis.transpose() << ")^T" << endl;
  cout << "Rotation matrix of Rodrigues' =" << endl;
  cout << rotation_matrix_rodrigues(angle, axis) << endl;
  cout << "Rotation matrix from library =" << endl;
  cout << rotation_matrix_library(angle, axis) << endl;
  
  AngleAxisd rotation_vector ( angle, axis );
  Quaterniond q = Quaterniond ( rotation_vector );
  
  // initialize q1 and q2 as random unit quaternions
  Quaterniond q1 = Quaterniond::UnitRandom();
  Quaterniond q2 = Quaterniond::UnitRandom();
  // coeffs is presented as (x,y,z,w), with imaginary part first and real part last
  cout << "q1 = (" << q1.coeffs().transpose() << ")^T" << endl;
  cout << "q2 = (" << q2.coeffs().transpose() << ")^T" << endl;
  cout << "q1 mult q2  = \n" << (q1*q2).coeffs() << endl;
  Vector4d q2v = q2.coeffs();
  Matrix4d q1m_1 = q_op_plus(q1.coeffs(), true);
  cout << "q1^ +  q2 = \n" << q_op_plus(q1.coeffs(), true) * q2.coeffs() << endl;
  cout << "q1^(+) q2 = \n" << q_op_plus(q1.coeffs(), false) * q2.coeffs() << endl;
  
  // assign quaternion with rotation matrix
  Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
  q = Quaterniond ( rotation_matrix );
  cout << "quaternion = \n" << q.coeffs() <<endl;
  // rotate a vector with quaternion
  Vector3d v ( 1,0,0 );
  Vector3d v_rotated = q * v; // overloaded mult, which is qvq^{-1}
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

}
