#ifndef _MECHLIB_HPP_
#define _MECHLIB_HPP_

#define DEFAULTMOVEKP 1
#define DEFAULTMOVEKD 0.0
#define MAX_POW 127

void baseControl (void * ignore);
void moveBase(double targ, double cutoff, double baseKp, double baseKd);
void moveBase(double targ, double cutoff);
void turnBase(double turn_bearing, double cutoff, double baseKp, double baseKd);
void turnBase(double bearing, double cutoff);

void toggleSwitch();
void toggleTilter(void *ignore);
void liftControl (void * ignore);
void moveLift (double lPosTarg);

void lcToggle();


#endif
