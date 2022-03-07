#include "main.h"

// VARIABLES
double leftEncoderDiff = 0;
double rightEncoderDiff = 0;
double backEncoderDiff = 0;

double currentBearing = 0;
double globalY = 0;
double globalX = 0;

double targetRotate = 0;
double cutoffRotate = 0;
bool baseRotateState = false;


// ODOMETRY
void baseOdometry(void * ignore) {
  // Set variables
  double prevLeftEncoder = 0;
  double prevRightEncoder = 0;
  double prevBackEncoder = 0;

  while (true) {
    // Distance moved by encoders
    leftEncoderDiff = prevLeftEncoder;
    rightEncoderDiff = prevLeftEncoder;
    backEncoderDiff = prevLeftEncoder;
    double theta = (leftEncoderDiff - rightEncoderDiff)/(SL + SR);
    currentBearing += theta;

    // Update prev variables
    prevLeftEncoder = 0;
    prevRightEncoder = 0;
    prevBackEncoder = 0;

    // Calculate local offset
    double arcRadius = (rightEncoderDiff/theta) + SR;
    double localXChange = 2 * (sin(theta/2)) * ((backEncoderDiff/theta) + SS);
    double localYChange = 2 * (sin(theta/2)) * arcRadius;

    // Calculate global offset by avg rotation
    double avgRotation = currentBearing - (theta/2);
    double globalXChange = (cos(-avgRotation) * localXChange) - (sin(-avgRotation) * localYChange);
    double globalYChange = (sin(-avgRotation) * localXChange) + (cos(-avgRotation) * localYChange);
    globalX += globalXChange;
    globalY += globalYChange;

    delay(5);
  }
}


// ROTATE CODE
void baseControl(void * ignore) {
  Motor FrontL (FRONTLPORT);
  Motor FrontR (FRONTRPORT);
  Motor MidL (MIDLPORT);
  Motor MidR (MIDRPORT);
  Motor BackL (BACKLPORT);
  Motor BackR (BACKRPORT);

  // ROTATE CODE
  if (baseRotateState) {

    // Initialise
    double targetRotate = targetRotate * (atan(1)*4/180);
    double prevErrorRotate = 0;
    double startTime = millis();

    // PD loop
    while (fabs(targetRotate - currentBearing) > ROTATEERRORMARGIN && millis() - startTime <= cutoffRotate) {

      // Calculate powers
      double errorRotate = targetRotate - currentBearing;
      double rotateProportional = errorRotate * ROTATEKP;
      double rotateDerivative = (errorRotate - prevErrorRotate) * ROTATEKD;
      double rotatePower = rotateProportional + rotateDerivative;
      double prevRotateError = errorRotate;

      if (rotatePower > ROTATEMAXPOWER) {
        rotatePower = ROTATEMAXPOWER;
      }
      if (rotatePower < ROTATEMAXPOWER * -1) {
        rotatePower = ROTATEMAXPOWER * -1;
      }

      // Move motors
      FrontL.move(rotatePower);
      MidL.move(rotatePower);
      BackL.move(rotatePower);
      FrontR.move(-rotatePower);
      MidR.move(-rotatePower);
      BackR.move(-rotatePower);

      delay(5);
    }
    baseRotateState = false;
  }

}


void rotateBase(double rotateBaseDegrees, double rotateBaseCutoff) {
  targetRotate = rotateBaseDegrees * (atan(1)*4/180);
  cutoffRotate = rotateBaseCutoff;
  baseRotateState = true;
}
