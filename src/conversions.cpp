#include "main.h"

// DEGREES TO RADIAN
double degreesToRadian(double degrees) {
  double radian = degrees * (atan(1)*4/180);
  return radian;
}
