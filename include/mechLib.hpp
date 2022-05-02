#ifndef _MECHLIB_HPP_
#define _MECHLIB_HPP_


#define ARMHIGH 500
#define ARMMID 230
#define ARMLOW -30
#define ARMKP 1

#define CLAWUP 0
#define CLAWDOWN -290
#define CLAWKP 0.35

#define DEFAULTDRIVEKP 0.3
#define DEFAULTDRIVEKD 0.2


void armControl(double height, double armKp);
void clawControl(double clawPosition);
void drivePD(void * ignore);
void baseMove(double targL, double targR, double cutoff, double kP, double kD);


#endif
