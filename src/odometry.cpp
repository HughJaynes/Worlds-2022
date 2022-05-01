#include "main.h"


double angle = HALFPI;

double globalX = 0, globalY = 0, prevLeftEncoder = 0, prevRightEncoder = 0;
double encdL = 0, encdR = 0, bearing = 0;

double boundRad(double rad){
  double res = fmod(rad, TWOPI);
  if(res < 0) res += TWOPI;
  return res;
}
double boundDeg(double deg){
  double res = fmod(deg, 360);
  if(res < 0) res += 360;
  return res;
}
double abscap(double x, double abscap){
  if(x > abscap) return abscap;
  else if(x < -abscap) return -abscap;
  else return x;
}



void getBearing(void*ignore) {
  Motor FL (FLPORT);
  Motor BL (BLPORT);
  Motor FR (FRPORT);
  Motor BR (BRPORT);
  Imu IMU (IMUPORT);

  double bearing = IMU.get_heading();
while(true){
  if (!IMU.is_calibrating()) {
    encdL = FL.get_position();
      encdR = FR.get_position();
      bearing = IMU.get_rotation();
      angle = HALFPI - bearing * DEGTORAD;
  }
  delay (5);
}
}


void baseOdometry(void * ignore) {

  Motor FL (FLPORT);
  Motor FR (FRPORT);

  Imu IMU (IMUPORT);
  double prevBearing = bearing;

  while (true) {

    if (IMU.is_calibrating()) {
      globalX = 0;
      globalY = 0;
    }
    else {
      double leftEncoderDiff = encdL - prevLeftEncoder;
      double rightEncoderDiff = encdR - prevRightEncoder;

      double distance = (leftEncoderDiff + rightEncoderDiff)/2*INPERDEG;
      globalX += distance*cos(angle);
      globalY += distance*sin(angle);

      prevLeftEncoder = encdL;
      prevRightEncoder = encdR;

      delay(5);
    }
  }

}
