#ifndef LIB_MATH_HPP
#define LIB_MATH_HPP

#include <cmath>

// cross product
template<typename T>
inline void cross(const T x[3], const T y[3], T result[3]){
  result[0] = x[1] * y[2] - x[2] * y[1];
  result[1] = x[2] * y[0] - x[0] * y[2];
  result[2] = x[0] * y[1] - x[1] * y[0];
}

// inner product
template<typename T>
inline T dot(const T x[3], const T y[3]){
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

// square norm
template<typename T>
inline T sqr_norm(const T x[3]){
  return dot<T>(x, x);
}

// angle axis rotation with point
template<typename T>
inline void aa_rot_pt(const T aa[3], const T pt[3], T out[3]){
  T theta = sqrt(sqr_norm(aa));
  if(theta > T(0)){
    // if theta is away from 0, use Rodrigues' formula
    // w = aa / theta
    // out = pt*cos(theta) + (w x pt)*sin(theta) + w*(w . pt)*(1 - cos(theta))
    const T w[3] = {aa[0]/theta, aa[1]/theta, aa[2]/theta};
    const T costh = cos(theta);
    const T sinth = sin(theta);
    T w_cross_pt[3];
    cross(w, pt, w_cross_pt);
    const T tmp = dot(w, pt) * (T(1) - costh);
    for(int i=0; i<3; i++){
      out[i] = pt[i] * costh + w_cross_pt[i] * sinth + w[i] * tmp;
    }
  }else{
    // if theta is 0, use tayler expansion to avoid division by zero
    // R = I + hat(w) * sin(theta)
    // since sin(theta) ~ theta and theta * w = aa, which gives us
    // R ~ I + hat(w) * theta
    // and actually performing multiplication with the point pt, gives us
    // out = R * pt = pt + aa x pt.
    T aa_cross_pt[3];
    cross(aa, pt, aa_cross_pt);
    for(int i=0; i<3; i++){
      out[i] = pt[i] + aa_cross_pt[i];
    }
  }
}


#endif
