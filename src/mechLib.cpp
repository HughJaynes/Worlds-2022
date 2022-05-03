#include "main.h"

#define MAX_POWL 127
#define MAX_POWR 100
#define RAMP_POW 0.5

int lPos = 0;
bool tPos = false;
bool cPos = false;
int rPos = 0;

int mode;
double bearing;
int targetL = 0, targetR = 0, targetRotate = 0;
double driveKp = 0, driveKd = 0, rotateKp = 0, rotateKd = 0;
double errorL = 0, errorR = 0, errorRotate = 0;
double derivativeL = 0, derivativeR = 0, derivativeRotate = 0;
double deltapowerL = 0, deltapowerR = 0, powerRotate = 0;
double previousL = 0, previousR = 0, previousRotate = 0;
int timeout = 0, startTime = 0;
double powerL = 0, powerR = 0;

/**
 * Lift controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the subsystemControl task
 */
void changeLiftUp () {
    if (lPos <= LIFTDOWN) {
        lPos = LIFTMID;
    } else if (lPos == LIFTMID) {
        lPos = LIFTUP;
    }
}

/**
 * Lift controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the subsystemControl task
 */
void changeLiftDown () {
    if (lPos == LIFTMID && rPos == 0) {
        lPos = LIFTDOWN;
    } else if (lPos == LIFTUP) {
        lPos = LIFTMID;
    } else if (lPos == LIFTMID) {
        changeRingOnOff();
        lPos = LIFTDOWN;
    }
}

/**
 * Tilter controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the subsystemControl task
 */
void changeTilter () {
    tPos = !tPos;
}

/**
 * Clamp controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the subsystemControl task
 */
void changeClamp () {
    cPos = !cPos;
}

/**
 * Ring intake controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the subsystemControl task
 */
void changeRingOnOff () {
    if (rPos == 0) {
        rPos = 1;
    } else if (rPos == 1 | rPos == 2) {
        rPos = 0;
    }
}

/**
 * Ring intake controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the subsystemControl task
 */
void changeRingUpDown () {
    if (rPos == 1) {
        rPos = 2;
    } else if (rPos == 2) {
        rPos = 1;
    }
}

/**
 * Subsystem controller function. 
 *
 * Intended to be run as a task throughout entire program, 
 * will control position of subsystems when variables are changed. 
 *
 * Takes no parameters. 
 */
void subsystemControl (void* ignore) {
	Motor LI (LIPORT);
    Motor RI (RIPORT);
    LI.tare_position();
    
    ADIDigitalOut LC (LCPISTON);
    ADIDigitalOut T1 (T1PISTON);
    ADIDigitalOut T2 (T2PISTON);
    ADIDigitalOut TC (TCPISTON);

    while (true) {
        LI.move(lPos - (LI.get_position()) * LIFTKP);
        if (tPos) {
            T1.set_value(tPos);
            T2.set_value(tPos);
            delay(200);
            TC.set_value(!tPos);
        }
        else {
            TC.set_value(!tPos);
            delay(200);
            T1.set_value(tPos);
            T2.set_value(tPos);
        }
        LC.set_value(cPos);
        if (rPos == 0) {
            RI.move(0);
        } else if (rPos == 1) {
            RI.move(RINGSPEED);
        } else if (rPos == 2) {
            RI.move(-RINGSPEED);
        }
        delay(5);
    }
}

/**
 * Base controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the baseControl task
 */
void moveBase (int lTarg, int rTarg, int customTimeout, double customKp, double customKd) {
    targetL = lTarg, targetR = rTarg;
    driveKp = customKp, driveKd = customKd;
    timeout = customTimeout;
    mode = MOVE_BASE;
}

/**
 * Base controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the baseControl task
 */
void rotateBase (int targ, int customTimeout, double customKp, double customKd) {
    targetRotate = targ;
    rotateKp = customKp, rotateKd = customKd;
    timeout = customTimeout;
    mode = ROTATE_BASE;
}

/**
 * Base controller function. 
 *
 * Intended to be run as a task throughout entire program, 
 * will control position of base when variables are changed. 
 *
 * Takes in no parameters. 
 */
void baseControl (void * ignore) {
    Motor FL(FLPORT);
    Motor ML(MLPORT);
	Motor BL(FLPORT);
	Motor FR(FRPORT);
    Motor MR(MRPORT);
	Motor BR(BRPORT);
    Imu IMU(IMUPORT);

    printf("PID task started");

    while(!competition::is_autonomous()) {
        delay(5);
    }

    while(competition::is_autonomous()){
        if (mode == MOVE_BASE) {
            FL.move(0);
            FL.tare_position();
            ML.move(0);
            ML.tare_position();
            BL.move(0);
            BL.tare_position();
            FR.move(0);
            FR.tare_position();
            MR.move(0);
            MR.tare_position();
            BR.move(0);
            BR.tare_position();
            // Movement Code
            errorL = targetL - (FL.get_position());
            errorR = targetR - (FR.get_position());
            startTime = millis();
            while ((fabs(errorL) > 40 || fabs(errorR) > 40) && (millis() - startTime) < timeout) {

                errorL = targetL - FL.get_position();
                errorR = targetR - FR.get_position();
                derivativeL = errorL - previousL;
                derivativeR = errorR - previousR;
                previousL = errorL;
                previousR = errorR;
                powerL = (errorL * driveKp) + (derivativeL * driveKd);
                powerR = (errorR * driveKp) + (derivativeR * driveKd);
                
                /*
                powerL = abscap(deltapowerL, RAMP_POW);
                powerR = abscap(deltapowerR, RAMP_POW);
                */

                FL.move (abscap(powerL,MAX_POWL));
                ML.move (abscap(powerL,MAX_POWL));
                BL.move (abscap(powerL,MAX_POWL));
                FR.move (abscap(powerR,MAX_POWR));
                MR.move (abscap(powerR,MAX_POWR));
                BR.move (abscap(powerR,MAX_POWR));
                delay(5);
                printf("DRIVE BASE: powerL: %.2f, errorL: %.2f, powerR: %.2f, errorR: %2.f\n", powerL, errorL, powerR, errorR);
            }
            mode = BRAKE_BASE;
        } else if (mode == ROTATE_BASE) {
            printf("ROTATE BASE");
            // Rotation Code
            bearing = IMU.get_heading();
            errorRotate = targetRotate - bearing;
            startTime = millis();
            while (fabs(errorRotate) > 20 && (millis() - startTime) < timeout) {
                bearing = IMU.get_heading();
                errorRotate = targetRotate - bearing;
                derivativeRotate = errorRotate - previousRotate;
                previousRotate = errorRotate;
                powerRotate = (errorRotate * rotateKp) + (derivativeRotate * rotateKd);
                FL.move(powerRotate);
                ML.move(powerRotate);
                BL.move(powerRotate);
                FR.move(-powerRotate);
                MR.move(-powerRotate);
                BR.move(-powerRotate);
                delay(5);
                printf("ROTATE BASE: bearing: %.2f, error: %.2f, power: %.2f\n", bearing, errorRotate, powerRotate);
            }
            mode = BRAKE_BASE;
        } else if (mode == BRAKE_BASE) {
            FL.move(0);
            FL.tare_position();
            ML.move(0);
            ML.tare_position();
            BL.move(0);
            BL.tare_position();
            FR.move(0);
            FR.tare_position();
            MR.move(0);
            MR.tare_position();
            BR.move(0);
            BR.tare_position();
            bearing = 0;
            targetL = 0, targetR = 0, targetRotate = 0;
            driveKp = 0, driveKd = 0, rotateKp = 0, rotateKd = 0;
            errorL = 0, errorR = 0, errorRotate = 0;
            derivativeL = 0, derivativeR = 0, derivativeRotate = 0;
            powerL = 0, powerR = 0, powerRotate = 0;
            previousL = 0, previousR = 0, previousRotate = 0;
            timeout = 0, startTime = 0;
            delay(5);
            printf("BRAKE BASE powerL: %.2f, errorL: %.2f, powerR: %.2f, errorR: %2.f\n", powerL, errorL, powerR, errorR);
        }
    }
}