#include "squirrel_object_manipulation/math_utils.hpp"

using namespace std;

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




