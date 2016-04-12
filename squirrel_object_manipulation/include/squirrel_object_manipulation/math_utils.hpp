#ifndef MATHUTILS
#define MATHUTILS

#include <math.h>
#include <stdlib.h>
#include <armadillo>

int sign(const double z);

double distancePoints(double x1, double y1, double z1, double x2, double y2, double z2);
double distancePoints(double x1, double y1, double x2, double y2);
double angle3Points(double x1, double y1, double x2, double y2, double x3, double y3);
double angle3PointsAbs(double x1, double y1, double x2, double y2, double x3, double y3);
double distance2Line(double x0, double y0, double x1, double y1, double x2, double y2);

arma::vec closestPointOnLine(double x0, double y0, double x1, double y1, double x2, double y2);
arma::vec rotate2DVector(double x, double y, double angle);
arma::vec rotate2DVector(arma::vec vec_, double angle);
arma::vec reflectPointOverPoint(double x0, double y0, double x1, double y1);
arma::vec pointsOnLineWithDistanceFromPoint(double x0, double y0, double x1, double y1, double d);
arma::vec pointOnLineWithDistanceFromPointInner(double x1, double y1, double x2, double y2, double d);
arma::vec pointOnLineWithDistanceFromPointOuter(double x1, double y1, double x2, double y2, double d);

double rotationDifference(double angle, double theta_robot);
double getVectorAngle(double x, double y);
double getNorm(arma::vec v);

double getGaussianVal(double x, double sigma, double mi);



#endif
