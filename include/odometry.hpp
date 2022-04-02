#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_


#define SL 7.25
#define SR 7.25
#define SS 7.5

#define ROTATEERRORMARGIN 0.1
#define ROTATEKP 400
#define ROTATEKD 200
#define ROTATEMAXPOWER 110

#define MOVEERRORMARGIN 1
#define MOVEKP 200
#define MOVEXCHANGER 1

void baseOdometry(void * ignore);
void baseControl(void * ignore);
void rotateBase(double rotateBaseDegrees, double rotateBaseCutoff);


#endif