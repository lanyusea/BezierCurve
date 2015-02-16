#include <stdio.h>
#include <gsl/gsl_vector.h>
typedef struct waypoint{
    double coordinate[3];
    gsl_vector *vector;
    double theta,phi,rho,sigma;
    double tensionIn;
    double tensionOut;
    struct waypoint *next;
    struct controlPoint *firstControl;
    struct controlPoint *secondControl;
} waypoint;

typedef struct controlPoint {
    double coordinate[3];
} controlPoint;


