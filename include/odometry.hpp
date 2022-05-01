#ifndef _ODOMETRY_
#define _ODOMETRY_


#define PI      3.14159265358979323846264338328
#define HALFPI  1.57079632679489661923132169164
#define TWOPI	  6.28318530717958647692528676656

#define RADTODEG 57.2957795130823208767981548141
#define DEGTORAD 0.0174532925199432957692369076849
#define INPERDEG 0.0461916735714906

double boundRad(double rad);
double boundDeg(double deg);
double abscap(double x, double abscap);

extern double encdL, encdR, bearing, angle;

void getBearing(void * ignore);
void baseOdometry(void * ignore);

#endif
