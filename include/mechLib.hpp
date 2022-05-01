#ifndef _MECHLIB_HPP_
#define _MECHLIB_HPP_

void liftControl (void* ignore);
void changeLift (int lPosTarget);

void baseControl (void* positionPointers);
void moveBase (int x, int y, int customCutoff, int margin, double customKp, double customKd);
void rotateBase (int targetBearing, int customCutoff, int margin, double customKp, double customKd);
void pointBase (double x, double y, int customCutoff, int margin, double customKp, double customKd);

#endif