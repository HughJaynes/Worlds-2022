#include "main.h"

double max_pow = 0;

int lPos = 0;
bool tPos = false;
bool tPosPrev = true;
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
    tPosPrev = tPos;
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
    if (rPos == 0 && lPos >= LIFTMID) {
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
    Motor FL(FLPORT);
    Motor ML(MLPORT);
	Motor BL(FLPORT);
	Motor FR(FRPORT);
    Motor MR(MRPORT);
	Motor BR(BRPORT);
    ADIDigitalOut LC (LCPISTON);
    ADIDigitalOut T1 (T1PISTON);
    ADIDigitalOut T2 (T2PISTON);
    ADIDigitalOut TC (TCPISTON);

    while (true) {
        // printf("Avg Base Temp: %10.f, Ring Temp: %10.f, Lift Temp: %10.f\n", (Fp.get_temperature() + FR.get_temperature() + ML.get_temperature() + MR.get_temperature() + BL.get_temperature() + BR.get_temperature()) / 6, RI.get_temperature(), LI.get_temperature());
        LI.move(lPos - (LI.get_position()) * LIFTKP);    
        if (tPos && !tPosPrev) {
            T1.set_value(HIGH);
            T2.set_value(HIGH);
            delay(500);
            TC.set_value(LOW);
            delay(750);
            TC.set_value(HIGH);
        } else if (!tPos && tPosPrev) {
            TC.set_value(HIGH);
            delay(250);
            T1.set_value(LOW);
            T2.set_value(LOW);
        }
        LC.set_value(cPos);
        if (rPos == 0) {
            RI.move(0);
        } else if (rPos == 1) {
            RI.move(RINGSPEED);
        } else if (rPos == 2) {
            RI.move(-RINGSPEED);
        }
        tPosPrev = tPos;
        delay(5);
    }
}

/**
 * Base controller interface
 * 
 * May be accessed from outside mechLib, 
 * so that the change in position may be passed onto the baseControl task
 */
void moveBase (int lTarg, int rTarg, int customTimeout, double customKp, double customKd, double maxPower) {
    targetL = lTarg, targetR = rTarg;
    driveKp = customKp, driveKd = customKd;
    timeout = customTimeout;
    mode = MOVE_BASE;
    max_pow = maxPower;
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
            FL.tare_position();
            ML.tare_position();
            BL.tare_position();
            FR.tare_position();
            MR.tare_position();
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
                FL.move (abscap(powerL,max_pow));
                ML.move (abscap(powerL,max_pow));
                BL.move (abscap(powerL,max_pow));
                FR.move (abscap(powerR,max_pow));
                MR.move (abscap(powerR,max_pow));
                BR.move (abscap(powerR,max_pow));
                delay(5);
            }
            mode = BRAKE_BASE;
        } else if (mode == ROTATE_BASE && !IMU.is_calibrating()) {
            printf("ROTATE BASE");
            // Rotation Code
            bearing = IMU.get_heading();
            errorRotate = targetRotate - bearing;

            startTime = millis();
            while (fabs(errorRotate) > 20 && (millis() - startTime) < timeout) {
                bearing = IMU.get_heading();
                errorRotate = targetRotate - bearing;
                if (errorRotate > 180) {
                    errorRotate = errorRotate - 360;
                }
                if (errorRotate < -180) {
                    errorRotate += 360;
                }
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
        }
    }
}