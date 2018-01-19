#include "squirrel_push_manipulation/math_utils.hpp"

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

    double alpha = a32 - a12;
    if(alpha < -2*M_PI) alpha = alpha + 2*M_PI;
    if(alpha > 2*M_PI) alpha = alpha - 2*M_PI;
    return alpha;
}

// distance of point 0 from line defined by points 1 and 2
double distance2Line(double x0, double y0, double x1, double y1, double x2, double y2){

    return fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / distancePoints(x1, y1, x2, y2);
}

// distance of point 0 from segment defined by points 1 and 2
double distance2Segment(double x0, double y0, double x1, double y1, double x2, double y2){
    vec p_close = closestPointOnLine(x0, y0, x1, y1,  x2, y2);
    if (p_close(0)<std::min(x1,x2) || p_close(0)>std::max(x1,x2) || p_close(1)<std::min(y1,y2) || p_close(1)>std::max(y1,y2))
        return 0.0;
    else
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

    //    if(err_th > 2 * M_PI) err_th = - 2 * M_PI + err_th;
    //    if(err_th < -2 * M_PI) err_th = 2 * M_PI + err_th;


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

double getAngle(arma::vec v1, arma::vec v2){

    return acos((v1(0) * v2(0) + v1(1) * v2(1))/(getNorm(v1)*getNorm(v2)));
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

vec pointsOnLineWithDistanceFromPoint(double x1, double y1, double x2, double y2, double d){
    //slope
    double m = (y2 - y1)/(x2 - x1);
    //intercept  line
    double b = y1 - m * x1;

    vec points(4);
    points(0) = x1 + d / sqrt(1 + m * m);
    points(1) = m * points(0) + b;
    points(2) = x1 - d / sqrt(1 + m * m);
    points(3) = m * points(2) + b;

    return points;

}

vec pointOnLineWithDistanceFromPointInner(double x1, double y1, double x2, double y2, double d){

    vec points = pointsOnLineWithDistanceFromPoint(x1, y1, x2, y2, d);
    vec point(2);

    if (distancePoints(points(0), points(1), x2, y2) < distancePoints(points(2), points(3), x2, y2)){
        point(0) = points(0);
        point(1) = points(1);
    }
    else{
        point(0) = points(2);
        point(1) = points(3);
    }

    return point;
}

vec pointOnLineWithDistanceFromPointOuter(double x1, double y1, double x2, double y2, double d){
    vec points = pointsOnLineWithDistanceFromPoint(x1, y1, x2, y2, d);
    vec point(2);

    if (distancePoints(points(0), points(1), x2, y2) > distancePoints(points(2), points(3), x2, y2)){
        point(0) = points(0);
        point(1) = points(1);
    }
    else{
        point(0) = points(2);
        point(1) = points(3);
    }

    return point;
}

double parallelCurveWidthTrans(double x, double dx, double y, double dy, bool sign, double w){

    if (dx == 0) dx = 0.001;
    if (dy == 0) dy = 0.001;
    if (sign)
        return x + w * dy / sqrt(dx * dx + dy * dy);
    else
        return x - w * dy / sqrt(dx * dx + dy * dy);
}

double circR(arma::vec alpha, arma::vec w){


    /*References:
  Statistical analysis of circular data, N.I. Fisher
   Topics in circular statistics, S.R. Jammalamadaka et al.
   Biostatistical Analysis, J. H. Zar
   Circular Statistics Toolbox for Matlab
*/

    // compute weighted sum of cos and sin of angles
    double Im = sum(w % sin(alpha));
    double Re = sum(w % cos(alpha));

    // obtain length
    return distancePoints(Im, Re, 0, 0)/sum(w);
}

double circKappa(arma::vec alpha, arma::vec w){

    //get mean
    double R = circR(alpha, w);

    //aproximate kappa
    // ref:  Statistical analysis of circular data, Fisher, equation p. 88

    double kappa = 0;
    if (R < 0.53){
        kappa = 2.0 * R + pow(R,3.0) + 5.0 * pow(R,5.0) / 6.0;
    }
    else if (R >= 0.53 && R < 0.85){
        kappa = -0.4 + 1.39 * R + 0.43 / (1 - R);
    }
    else{
        kappa = 1.0 / (pow(R,3.0) - 4*pow(R,2.0) + 3.0 * R);
    }

    int N = alpha.size();

    if (N < 15){
        if (kappa < 2){
            kappa = max(kappa - 2 / (N * kappa), 0.0);
        }
        else{
            kappa = pow(N-1,3.0) * kappa / (pow(N,3.0) + N);
        }

    }

    return kappa;
}

arma::vec getVMParam(arma::vec alpha, arma::vec w){

    arma::vec param(2);
    //mu
    param(0) = circMean(alpha,w);

    //kappa
    param(1) = circKappa(alpha,w);

    return param;
}

double circMean(arma::vec alpha, arma::vec w){
    //compute weighted sum of cos and sin of angles
    double Im = sum(w % sin(alpha));
    double Re = sum(w % cos(alpha));

    // obtain mean by
    double mu = atan2(Im, Re);

    return mu;

}

arma::vec sampleVM(arma::vec param, int N){

    /*References:
   Statistical analysis of circular data, Fisher, sec. 3.3.6, p. 49
   Circular Statistics Toolbox for Matlab
*/
    double theta = param(0);
    double kappa = param(1);

    srand ( time(NULL) );

    arma::vec alpha(N);
    // if kappa is small, treat as uniform distribution
    if (kappa < 1e-6){
        for (int i=0; i<N; i++){
            alpha(i) = 2 * M_PI * ((double) rand() / (RAND_MAX));
        }
    }
    return alpha;


    double a = 1 + sqrt(1 + 4 * pow(kappa,2));
    double b = (a - sqrt(2 * a)) / (2 * kappa);
    double r = (1 + b * b) / (2 * b);

    double z, f, c, u1, u2, u3;
    for (int i=0; i<N; i++){
        while(1){

            u1 = ((double) rand() / (RAND_MAX));
            u2 = ((double) rand() / (RAND_MAX));
            u3 = ((double) rand() / (RAND_MAX));

            z = cos(M_PI * u1);
            f = (1 + r * z) / ( r + z);
            c = kappa * (r-f);

            if ((u2 < c * (2-c) )|| !(log(c)-log(u2) + 1 -c < 0)) break;
        }

        alpha(i) = theta +  sign(u3 - 0.5) * acos(f);
        alpha(i) = atan2(sin(alpha(i)), cos(alpha(i)));
    }

    return alpha;

}

double getVMval(double alpha, arma::vec param){
 // get von Mises pdf val

    double theta = param(0);
    double kappa = param(1);

    double C = 1/(2 * M_PI *  boost::math::cyl_bessel_i(0,kappa));
    double p = C * exp(kappa * cos(alpha - theta));
    return p;

}
