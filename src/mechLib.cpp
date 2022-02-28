#include "main.h"
#include "conversions.cpp"

// VARIABLES
double leftEncoderReading = 0;
double rightEncoderReading = 0;
double backEncoderReading = 0;
double currentBearing = 0;
double globalY = 0;
double globalX = 0;
double rotateDegrees = 0;
double rotateCutoff = 0;
bool baseRotateState = false;
double targetX = 0;
double targetY = 0;
double moveCutoff = 0;
bool baseMoveState = false;

// ODOMETRY
void baseOdometry(void * ignore) {
  while (true) {
    // ENCODER READINGS
    leftEncoderReading = 0;
    rightEncoderReading = 0;
    backEncoderReading = 0;

    // CALCULATE ANGLE AND GLOBAL CHANGE
    double theta = (leftEncoderReading - rightEncoderReading)/(SL + SR);
    double arcRadius = (rightEncoderReading/theta) + SR;
    double localYChange = 2 * (sin(theta/2)) * arcRadius;
    double localXChange = 2 * (sin(theta/2)) * ((backEncoderReading/theta) + SS);
    double globalYChange = (sin(theta/2) * localXChange) + (cos(theta/2) * localYChange);
    double globalXChange = (cos(theta/2) * localXChange) - (sin(theta/2) * localYChange);
    currentBearing += theta;
    globalX += globalXChange;
    globalY += globalYChange;

    delay(5);
  }
}


// ROTATE CODE
void baseRotate(void * ignore) {
  Motor FrontL (FRONTLPORT);
  Motor FrontR (FRONTRPORT);
  Motor MidL (MIDLPORT);
  Motor MidR (MIDRPORT);
  Motor BackL (BACKLPORT);
  Motor BackR (BACKRPORT);

  // INITIALIZE
  double rotateRadians = degreeToRadian(rotateDegrees);
  double startTime = millis();
  double prevError = 0;

  // MOVEMENT CODE
  if (baseRotateState) {
    while ((fabs(rotateRadians - currentBearing) > ROTATEERRORMARGIN) && millis() - startTime <= rotateCutoff) {
      double rotateError = rotateRadians - currentBearing;
      double rotateP = rotateError * ROTATEKP;
      double rotateD = (rotateError - prevError) * ROTATEKD;
      double rotatePower = rotateP + rotateD;
      if (rotatePower > ROTATEMAXPOWER) {
        rotatePower = ROTATEMAXPOWER;
      }
      if (rotatePower < ROTATEMAXPOWER * -1) {
        rotatePower = ROTATEMAXPOWER * -1;
      }

      FrontL.move(rotatePower);
      MidL.move(rotatePower);
      BackL.move(rotatePower);
      FrontR.move(-rotatePower);
      MidR.move(-rotatePower);
      BackR.move(-rotatePower);

      delay(5);
    }
    FrontL.move(0);
    MidL.move(0);
    BackL.move(0);
    FrontR.move(0);
    MidR.move(0);
    BackR.move(0);
    baseRotateState = false;
  }
}


// BASE MOVE TO
void baseMove(void * ignore) {
  Motor FrontL (FRONTLPORT);
  Motor FrontR (FRONTRPORT);
  Motor MidL (MIDLPORT);
  Motor MidR (MIDRPORT);
  Motor BackL (BACKLPORT);
  Motor BackR (BACKRPORT);

  // INITIALISE
  double moveStartTime = millis();
  double movePrevError = 0;
  double errorLocalY = (targetY - globalY);
  double errorLocalX = targetX - globalX;

  // MOVEMENT CODE
  if (baseMoveState) {
    while () {



      delay(5);
    }

  }
}


void rotateBase(double rotateBaseDegrees, double rotateBaseCutoff) {
  rotateDegrees = rotateBaseDegrees;
  rotateCutoff = rotateBaseCutoff;
  baseRotateState = true;
}
