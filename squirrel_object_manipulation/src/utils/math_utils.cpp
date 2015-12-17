#include "squirrel_object_manipulation/math_utils.hpp"

using namespace std;

double distancePoints(double x1, double y1, double z1, double x2, double y2, double z2){

return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(y1-y2));
}

double distancePoints(double x1, double y1, double x2, double y2){

return distancePoints (x1, y1, 0, x2, y2, 0);
}




