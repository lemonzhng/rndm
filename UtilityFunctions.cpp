#include "UtilityFunctions.hpp"

int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
  // Replace these two coefficients with what you get
  // in the experiment. Note the trailing "f" after the
  // number -- this ensures that we use single precision
  // floating point (rather than double precision, which
  // would be substantially slower on the microcontroller).
  float a = -74.3f;  // the zeroth order term
  float b = 0.141f;  // the first order term
  //float a = -66.0f;
  //float b = 0.125f;

  return int(a + b * desiredSpeed_rad_per_sec);
}

float speedFromForce(float desiredForce_N) {
  // replace this with your determined constant:
  // Remember to add the trailing "f" for single
  // precision!
  float const propConstant = 2.836e-08f;
  //float const propConstant = 2.3e-08f;

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
    return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant);
}

