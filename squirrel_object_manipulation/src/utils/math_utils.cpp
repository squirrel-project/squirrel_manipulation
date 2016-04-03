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

    return a32 - a12;
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

//get vector norm

double getNorm(vec v){

    return distancePoints (v(0), v(1), 0, 0, 0, 0);
}


//reflection of point p1 over the point p2
vec reflectPointOverPoint(double x0, double y0, double x1, double y1){
    vec result(2);

    result(0) = 2 * x0 - x1;
    result(1) = 2 * y0 - y1;

    return result;

}


//angle difference for rotation
double rotationDifference(double angle, double theta_robot){

    double err_th = angle - theta_robot;

    if(err_th > M_PI) err_th = - (2 * M_PI - err_th);
    if(err_th < -M_PI) err_th = 2 * M_PI + err_th;

    return err_th;
}

// angle of a vector (x,y)
double getVectorAngle(double x, double y){

    double th = atan2(y,x);

    if (isnan(th)) th = 0.0;

    return th;
}

double getGaussianVal(double x, double sigma, double mi){
    double r = exp( - pow( x  - mi, 2) / (2 * pow(sigma, 2)));
    if (isnan(r)) r = 1.0;
    return r;
 }

int sign(const double z)
{
   return (z > 0.0) ? 1 : - 1;
}

//double pointOnLineWithDistanceFromPoint(double x1, double y1, double x2, double y2, double d){
//    //slope
//    double m = (y2 - y1)/(x2 - x1);
//    //intercept  line
//    double b = y1 - m * x1;

//    vec out = linecirc(m, b, x1, y1, d);

//    if  ( abs(norm([xout(1); yout(1)] - [x2; y2])) >  abs(norm([xout(2); yout(2)] - [x2; y2])))
//        p = [xout(1); yout(1)];
//    else
//    p = [xout(2); yout(2)];
//    end
//}

//vec flinecirc(slope,intercpt,centerx,centery,radius) {
//    /*finds
//%  the points of intersection given a circle defined by a center and
//%  radius in x-y coordinates, and a line defined by slope and
//%  y-intercept, or a slope of "inf" and an x-intercept.  Two points
//%  are returned.  When the objects do not intersect, NaNs are returned.
//%  When the line is tangent to the circle, two identical points are
//%  returned. All inputs must be scalars
//%
//*/

//    if (!isnan(slope))
//        // From the law of cosines

//        double a = 1 + slope^2;
//    double b=2 * (slope * (intercpt - centery) - centerx);
//    double c=centery^2 + centerx^2 + intercpt^2 - 2*centery.*intercpt-radius.^2;

//    x=roots([a,b,c])';

//            %  Make NaN's if they don't intersect.

//            if ~isreal(x)
//            x=[NaN NaN]; y=[NaN NaN];
//            else
//            y=[intercpt intercpt]+[slope slope].*x;
//            end

//            % vertical slope case
//            elseif abs(centerx-intercpt)>radius  % They don't intercept
//            x=[NaN;NaN]; y=[NaN;NaN];
//            else
//            x=[intercpt intercpt];
//            step=sqrt(radius^2-(intercpt-centerx)^2);
//            y=centery+[step,-step];
//            end


//}
