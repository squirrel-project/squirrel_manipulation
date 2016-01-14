#include "squirrel_object_manipulation/math_utils.hpp"

using namespace std;
using namespace arma;

double distancePoints(double x1, double y1, double z1, double x2, double y2, double z2){

    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(y1-y2));
}

double distancePoints(double x1, double y1, double x2, double y2){

    return distancePoints (x1, y1, 0, x2, y2, 0);
}


// angle defined with three points with point 2 as center

double angle3PointsAbs(double x1, double y1, double x2, double y2, double x3, double y3){


    double d12 = distancePoints(x1, y1, x2, y2);

    double d13 = distancePoints(x1, y1, x3, y3);

    double d23 = distancePoints(x3, y3, x2, y2);

    //angle point1 - point2 - point3

    double angle = acos((d12 * d12 + d23 * d23 - d13 * d13) / (2 * d12 * d23));
    if (angle > 3.14) angle = angle - 2*3.14;
    if (angle< -3.14) angle = angle + 2*3.14;
    if (isnan(angle)) angle = 0;

    return angle;
}

// angle defined with three points 1-2-3
double angle3Points(double x1, double y1, double x2, double y2, double x3, double y3){

    //angle between first and second point
    double a12 = atan2(y1 - y2, x1 -x2);
    if (isnan(a12)) a12 = 0;

    //angle between third and second point
    double a32 = atan2(y3 - y2, x3 -x2);
    if (isnan(a32)) a32 = 0;

    return a12 - a32;
}

// distance of point 0 from line defined by points 1 and 2
double distance2Line(double x0, double y0, double x1, double y1, double x2, double y2){

    return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / distancePoints(x1, y1, x2, y2);
}

vec closestPointOnLine(double x0, double y0, double x1, double y1, double x2, double y2){
    vec result(2);

    //slope first line
    double m1 = (y2 - y1)/(x2 - x1);
    //intercept first line
    double b1 = y1 - m1 * x1;

    //perpendicular line slope
    double m2 = - 1 / m1;
    //perpendicular line intercept - conatining x0
    double b2 = y0 - m2 * x0;

    //intersection point
    result(0) = -(b1 - b2)/(m1 - m2);
    result(1) = m2 * result(0) + b2;

    return result;

}

vec rotate2DVector(double x, double y, double angle){

    vec rot_vec_(2);

    rot_vec_(0) = x * cos(angle) - y * sin(angle);
    rot_vec_(1) = x * sin(angle) + y * cos(angle);

    return rot_vec_;
}
vec rotate2DVector(vec vec_, double angle){
    return rotate2DVector(vec_(0), vec_(1), angle);
}


//reflection of point p1 over the point p2
vec reflectPointOverPoint(double x0, double y0, double x1, double y1){
    vec result(2);

    result(0) = 2 * x0 - x1;
    result(1) = 2 * y0 - y1;

    return result;

}


//angle difference for rotation in first and fourth quadrant
double rotationDifference(double angle, double theta_robot){

//    if (angle < 0) angle = angle + 2 * M_PI;
//    if (theta_robot < 0) theta_robot = theta_robot + 2 * M_PI;

    double err_th = angle - theta_robot;

    if(err_th > M_PI) err_th = - (2 * M_PI - err_th);
    if(err_th < -M_PI) err_th = 2 * M_PI + err_th;

    return err_th;
}

// angle of a vector (x,y)
double getVectorAngle(double y, double x){

    double th = atan2(y,x);

    if (isnan(th)) th = 0;
    //if (th < 0) th = 2*M_PI + th;

    return th;
}




