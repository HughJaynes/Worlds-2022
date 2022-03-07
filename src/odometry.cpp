#include "main.h"
#include "conversions.cpp"

// VARIABLES
double leftEncoderDiff = 0;
double rightEncoderDiff = 0;
double backEncoderDiff = 0;

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
    leftEncoderDiff = prevLeftEncoder;
    rightEncoderDiff = prevLeftEncoder;
    backEncoderDiff = prevLeftEncoder;

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
  double targetRotate = degreeToRadian(rotateDegrees);
  double startTime = millis();
  double prevError = 0;

  // MOVEMENT CODE
  if (baseRotateState) {
    while (fabs(targetRotate - currentBearing) > ROTATEERRORMARGIN && millis() - startTime <= rotateCutoff) {
      double errorRotate = targetRotate - currentBearing;
      double rotateProportional = errorRotate * ROTATEKP;
      double rotateDerivative = (errorRotate - prevError) * ROTATEKD;
      double rotatePower = rotateProportional + rotateDerivative;
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
void baseCurveRight(void * ignore) {
  Motor FrontL (FRONTLPORT);
  Motor FrontR (FRONTRPORT);
  Motor MidL (MIDLPORT);
  Motor MidR (MIDRPORT);
  Motor BackL (BACKLPORT);
  Motor BackR (BACKRPORT);

  // CHANGE TO LOCAL
  double moveLocalTargetY = (sin(currentBearing/2) * globalX) + (cos(currentBearing/2) * globalY);
  double moveLocalTargetX = (cos(currentBearing/2) * globalX) - (cos(currentBearing/2) * globalY);

  // INITIALISE
  double moveStartTime = millis();
  double movePrevError = 0;
  double errorY = targetY - moveLocalTargetY;
  double errorX = targetX - moveLocalTargetX;

  // MOVEMENT CODE
  if (baseMoveState) {
    while (errorY > MOVEERRORMARGIN && errorX > MOVEERRORMARGIN && millis() - moveStartTime <= moveCutoff) {

      moveLocalTargetY = (sin(currentBearing/2) * globalX) + (cos(currentBearing/2) * globalY);
      moveLocalTargetX = (cos(currentBearing/2) * globalX) - (cos(currentBearing/2) * globalY);
      errorY = targetY - moveLocalTargetY;
      errorX = targetX - moveLocalTargetX;

      double proportionalL = errorY * MOVEKP;
      double proportionalR = errorY * MOVEKP;
      double derivative = errorY - movePrevError;
      double movePrevError = errorY;



      delay(5);
    }

  }
}


void rotateBasef(double rotateBaseDegrees, double rotateBaseCutoff) {
  rotateDegrees = rotateBaseDegrees;
  rotateCutoff = rotateBaseCutoff;
  baseRotateState = true;
}
