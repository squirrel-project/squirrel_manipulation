#ifndef MATHUTILS
#define MATHUTILS

#include <math.h>

double distancePoints(double x1, double y1, double z1, double x2, double y2, double z2);
double distancePoints(double x1, double y1, double x2, double y2);
double angle3Points(double x1, double y1, double x2, double y2, double x3, double y3);
double angle3PointsAbs(double x1, double y1, double x2, double y2, double x3, double y3);

#endif
