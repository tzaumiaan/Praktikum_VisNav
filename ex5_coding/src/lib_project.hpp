#ifndef LIB_PROJECT_HPP
#define LIB_PROJECT_HPP

#include <Eigen/Core>

#include "lib_math.hpp"

// cam: 9 dims array with 
// [0-2]: angle-axis rotation 
// [3-5]: translateion
// [6]: focal length
// [7-8]: second and forth order radial distortion
// pt : 3D presentation of a landmark point
// pred : 2D predictions with center of the image plane. 
template<typename T>
inline bool cam_project_w_dist(const T* cam, const T* pt, T* pred){
  T p[3];
  // apply camera rotation on a point
  aa_rot_pt(cam, pt, p);
  // apply camera translation on it
  for(int i=0; i<3; i++){
    p[i] += cam[3+i];
  }
  // perspective division from 3D to 2D x-y plan
  T px = -p[0]/p[2], py = -p[1]/p[2];
  // camera parameters
  const T& f = cam[6], k1 = cam[7], k2 = cam[8];
  // distortion parameters
  T r2 = px*px + py*py;
  T dist = T(1) + r2*k1 + r2*r2*k2;
  // output projected point
  pred[0] = f * dist * px;
  pred[1] = f * dist * py;

  return true;
}

// to show camera pose as angle axis with center point
// note: because of Eigen library, only double is supported as data type
typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
void cam_to_aa_ct(const double* cam, double* aa, double* ct){
  VectorRef angle_axis_ref(aa,3);
  angle_axis_ref = ConstVectorRef(cam,3);
  // center is the inverse rotation of camera translation
  // ct = -R' tr
  Eigen::VectorXd inverse_rotation = -angle_axis_ref;
  double cam_tr[3] = {cam[3], cam[4], cam[5]};
  aa_rot_pt(inverse_rotation.data(), cam_tr, ct);
  VectorRef(ct,3) *= -1.0;
}
#endif
