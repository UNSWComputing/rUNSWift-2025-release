#pragma once

#include <Eigen/Dense>
#include <cmath>

// Returns the minimum number of radians between two thetas.
// Ported from: utils/angles.hpp
inline static float minThetaDiff(float thetaA, float thetaB) {
   while (thetaA > M_PI) thetaA -= 2.0f*M_PI;
   while (thetaA < -M_PI) thetaA += 2.0f*M_PI;

   while (thetaB > M_PI) thetaB -= 2.0f*M_PI;
   while (thetaB < -M_PI) thetaB += 2.0f*M_PI;

   return fmin(fabs(thetaA - thetaB), fabs(fabs(thetaA - thetaB) - 2.0f*M_PI));
}

inline float deg2radf(float deg) {
    return deg * M_PI / 180.0;
}

inline static float normaliseTheta(float theta) {
   double r = fmod(theta - M_PI, 2.0*M_PI);
   if (r > 0) {
      return r - M_PI;
   } else {
      return r + M_PI;
   }
}
