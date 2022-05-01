#include "main.h"

int lPos = 0;

int mode = BRAKE_BASE;
int cutoff;
int errorMargin;
double Kp;
double Kd;
int xTarget;
int yTarget;
int bearingTarget;

int currError;
int prevError;
int proportional;
int derivative;
int power;
int startTime;

int* xPos;
int* yPos;
int* bearingPos;

/**
 * Lift controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the liftControl task
 */
void changeLift (int lPosTarget) {
    lPos = lPosTarget;
}

/**
 * Base controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the baseControl task
 */
void moveBase (int x, int y, int customCutoff = 1000, int margin = BASEERRORMARGIN, double customKp = BASEKP, double customKd = BASEKD) {
    xTarget = x;
    yTarget = y;
    cutoff = customCutoff;
    errorMargin = margin;
    Kp = customKp;
    Kd = customKd;
    mode = MOVE_BASE;
}

/**
 * Base rotation controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in bearing may be passed onto the baseControl task
 */
void rotateBase (int targetBearing, int customCutoff = 1000, int margin = ROTATEERRORMARGIN, double customKp = ROTATEKP, double customKd = ROTATEKD) {
    bearingTarget = targetBearing;
    cutoff = customCutoff;
    errorMargin = margin;
    Kp = customKp;
    Kd = customKd;
    mode = ROTATE_BASE;
}

/**
 * Base rotation controller interface (Pointing)
 * 
 * May be accessed from outside mechLib, 
 * so that the change in bearing may be passed onto the baseControl task
 */
void pointBase(double x, double y, int customCutoff = 1000, int margin = ROTATEERRORMARGIN, double customKp = ROTATEKP, double customKd = ROTATEKD) {
  double pointXError = x - *xPos;
  double pointYError = x - *yPos;
  double targetPoint = atan(pointXError/pointYError) * 180/atan(1)*4;
  rotateBase(targetPoint, customCutoff, margin, customKp, customKd);
}

/**
 * Lift controller function. 
 *
 * Intended to be run as a task throughout entire program, 
 * will control position of lift when variable lPos is changed. 
 *
 * Takes in one parameter; pointer to lPos to read the variable. 
 */
void liftControl (void* ignore) {
	Motor LI (LIPORT);
    LI.tare_position();

    while (true) {
        LI.move(lPos - (LI.get_position()) * LIFTKP);
        delay(5);
    }
}

/**
 * Base controller function. 
 *
 * Intended to be run as a task throughout entire program, 
 * will control position of base when targets are changed. 
 *
 * Takes in one parameter; pointer to pointers to read the targets and odom values. 
 */
void baseControl (void* positionPointers) {
    Motor FL (FLPORT);
    Motor FR (FRPORT);
    Motor ML (MLPORT);
    Motor MR (MRPORT);
    Motor BL (BLPORT);
    Motor BR (BRPORT);

    xPos = (int *)positionPointers;
    yPos = (int *)positionPointers + 1;
    bearingPos = (int *)positionPointers + 2;

    while (true) {
        if (competition::is_autonomous()) {
            if (mode == ROTATE_BASE) {
                // Rotation Code
                bearingTarget = bearingTarget * (atan(1)*4/180);
                prevError = 0;
                startTime = millis();

                // PD loop
                while (fabs(bearingTarget - *bearingPos) > errorMargin && millis() - startTime <= cutoff) {
                    // Calculate powers
                    currError = bearingTarget - *bearingPos;
                    proportional = currError * Kp;
                    derivative = (currError - prevError) * Kd;
                    power = proportional + derivative;
                    double prevError = currError;

                    // Move motors
                    FL.move(power);
                    ML.move(power);
                    BL.move(power);
                    FR.move(-power);
                    MR.move(-power);
                    BR.move(-power);

                    delay(5);
                }

                mode = BRAKE_BASE;
            } else if (mode == MOVE_BASE) {
                // Movement Code
                double moveXError = targetXMove - globalX;
                double moveYError = targetYMove - globalY;
                double straightError = sqrt(pow(moveXError,2) + pow(moveXError,2));
                double startTime = millis();

                //PD Loop
                while (fabs(straightError) > MOVEERRORMARGIN && millis() - startTime <= cutoffMove) {
                    double moveXError = targetXMove - globalX;
                    double moveYError = targetYMove - globalY;
                    double straightError = sqrt(pow(moveXError,2) + pow(moveXError,2));
                    delay(5);
                }
            }
            else if (mode == BRAKE_BASE) {
                FL.move(0);
                ML.move(0);
                BL.move(0);
                FR.move(0);
                MR.move(0);
                BR.move(0);
            }
        }
    }
}