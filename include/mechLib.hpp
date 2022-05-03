#ifndef _MECHLIB_HPP_
#define _MECHLIB_HPP_

void changeLiftUp();
void changeLiftDown();
void changeTilter();
void changeClamp();
void changeRingOnOff();
void changeRingUpDown();
void subsystemControl(void* ignore);
void moveBase (int lTarg, int rTarg, int customTimeout, double customKp = BASEKP, double customKd = BASEKD);
void rotateBase (int targ, int customTimeout, double customKp = ROTATEKP, double customKd = ROTATEKD);
void baseControl(void * ignore);

#endif
