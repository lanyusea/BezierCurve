//This is a C implement of Hobby's Algorithm(1996).
//
//Given a set of points(P1,P2...Pn), draw a Bezier's Curve which goes through
//every point.
//
//Points can be 2D or 3D but at least three points.
//Running example:
//      ./planning "(0,0,0);(1,1,1);(2,0,0);(1,-1,1)"
//      ./planning "(0,0,0);(1,1,1);(2,0,0);(1,-1,1)" "(0,0,0);(0,1,0);(1,1,-1);(0,0,0)"
//
//Different points should be separated by `;` and cited with`()`, different coordinate bits
//of the same point should be separated by `,`
//
//The result will be print out
//
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
//GNU Scientific Library for Linear Algebra Calculation
#include <gsl/gsl_sf_bessel.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
//Define bool type in C
typedef enum { false, true } bool;
//cycle determines whether the path is closed or not
bool cycle = true;

//Store waypoints to a linked list, and return the
//pointer of the first element.
waypoint* addWaypoint(char *rawData){
    char *str = rawData; //The input string.
    char *end_str;
    char *token = strtok_r(str, ";", &end_str); //Token stores string split results.

    //Construct the first node of linked list and allocate memory
    waypoint *pn = NULL;
    pn = malloc(sizeof(struct waypoint));
    waypoint *pprev = pn;

    while (token != NULL)
    {
        int i = 0;
        char *end_token;
        token ++;
        token[strlen(token) - 1] = 0;
        char *token2 = strtok_r(token, ",", &end_token);
        while (token2 != NULL)
        {
            pprev->coordinate[i] = atof(token2);//convert string into float number
            token2 = strtok_r(NULL, ",", &end_token);
            i ++;
        }

        //Construct next node and allocate memory
        waypoint *pnext;
        pnext       = malloc(sizeof(struct waypoint));
        pprev->next = pnext;
        pprev       = pnext;
        token       = strtok_r(NULL, ";", &end_str);
    }
    return pn;
}

//Arrange existing points and count total waypoints
int pointsArrangement(waypoint **wayp) {
    waypoint *p1 = *wayp; //Pointer of the first node of linked list
    waypoint *p0 = p1;
    int counter = 0;
    while (p1->next != NULL){
        counter ++;
        p1->vector          = gsl_vector_alloc(3);
        p1->firstControl    = malloc(sizeof(struct controlPoint)); //Allocate memory of control nodes struct
        p1->secondControl   = malloc(sizeof(struct controlPoint)); //Allocate memory of control nodes struct
        p1->tensionIn       = 1; //Set tension that path goes into this point
        p1->tensionOut      = 1; //Set tension that path goes ouf of this point

        p0 = p1;
        p1 = p1->next;
    }

    //If path is a closed circle, copy and put the first point into the last node of linked list
    //If not, remove the last node and free its memory
    if (cycle) {
        *p1 = **wayp;
        p0->next = p1;
        p1->next = NULL;
        p1->firstControl    = malloc(sizeof(struct controlPoint));
        p1->secondControl   = malloc(sizeof(struct controlPoint));
        counter ++;
    }
    else {
        free(p1);
        p0->next = NULL;
    }
    return counter; //Return total number of points
}

//Store direction vector from command arguments
int getVector(char *rawData, waypoint** wayp){
    char *str = rawData; //The input string.
    char *end_str;
    char *token = strtok_r(str, ";", &end_str); //Token stores string split results.

    waypoint* p1 = *wayp;
    int counter = 0;
    double norm;

    while (token != NULL)
    {
        int i = 0;
        char *end_token;
        token ++;
        token[strlen(token) - 1] = 0;
        char *token2 = strtok_r(token, ",", &end_token);
        while (token2 != NULL)
        {
            gsl_vector_set(p1->vector,i,atof(token2));
            token2 = strtok_r(NULL, ",", &end_token);
            i ++;
        }
        norm = gsl_blas_dnrm2(p1->vector);
        if (norm > 10e-7) gsl_vector_scale(p1->vector,(1/norm)); //NO NORMALIZATION for (0,0,0)
        p1 = p1->next;
        counter ++;
        token = strtok_r(NULL, ";", &end_str);
    }
    //copy the first vector into the last one if cycle
    if (cycle) gsl_vector_memcpy(p1->vector,(*wayp)->vector);
    return cycle?counter+1:counter;
}
//Calculate the direction vector at each point when path going through
void calculateVector(waypoint **wayp) {
    double norm;                    //Store the vector norm
    waypoint *p0        = *wayp;    //The first node of linked list
    waypoint *p1        = p0->next;
    waypoint *pSecond   = p1;       //The second node of linked list
    waypoint *p2        = p1->next;

    //Handle general case:
    //for i in range(2,-2)
    //  vector(i) = point (i+1) - point (i-1)
    //end
    while (p1->next != NULL) {
        //Do nothing is if has default direction vector
        if (gsl_blas_dnrm2(p1->vector) < 10e-7) {
            gsl_vector_set(p1->vector,0,p2->coordinate[0]-p0->coordinate[0]);
            gsl_vector_set(p1->vector,1,p2->coordinate[1]-p0->coordinate[1]);
            gsl_vector_set(p1->vector,2,p2->coordinate[2]-p0->coordinate[2]);

            norm = gsl_blas_dnrm2(p1->vector);
            gsl_vector_scale(p1->vector,(1/norm)); //Normalize the vector
        }

        p0 = p1;
        p1 = p2;
        p2 = p2->next;
    }

    //Handle special case:
    //if cycle
    //  vector(1) = point(2) - point(-2)
    //  vector(-1) = point(2) - point(-2)
    //elif not cycle
    //  vector(1) = point(2) - point(1)
    //  vector(-1) = point(-1) - point(-2)
    //end
    //
    //And do nothing if has default value
    if(cycle) {
        if (gsl_blas_dnrm2((*wayp)->vector) < 10e-7) {
            gsl_vector_set((*wayp)->vector,0,pSecond->coordinate[0]-p0->coordinate[0]);
            gsl_vector_set((*wayp)->vector,1,pSecond->coordinate[1]-p0->coordinate[1]);
            gsl_vector_set((*wayp)->vector,2,pSecond->coordinate[2]-p0->coordinate[2]);
            norm = gsl_blas_dnrm2((*wayp)->vector);
            gsl_vector_scale((*wayp)->vector,(1/norm));
        }

        if (gsl_blas_dnrm2(p1->vector) < 10e-7) {
            gsl_vector_set(p1->vector,0,pSecond->coordinate[0]-p0->coordinate[0]);
            gsl_vector_set(p1->vector,1,pSecond->coordinate[1]-p0->coordinate[1]);
            gsl_vector_set(p1->vector,2,pSecond->coordinate[2]-p0->coordinate[2]);
            norm = gsl_blas_dnrm2(p1->vector);
            gsl_vector_scale(p1->vector,(1/norm));
        }
    }
    else {
        if (gsl_blas_dnrm2((*wayp)->vector) < 10e-7) {
            gsl_vector_set((*wayp)->vector,0,pSecond->coordinate[0]-(*wayp)->coordinate[0]);
            gsl_vector_set((*wayp)->vector,1,pSecond->coordinate[1]-(*wayp)->coordinate[1]);
            gsl_vector_set((*wayp)->vector,2,pSecond->coordinate[2]-(*wayp)->coordinate[2]);
            norm = gsl_blas_dnrm2((*wayp)->vector);
            gsl_vector_scale((*wayp)->vector,(1/norm));
        }

        if (gsl_blas_dnrm2(p1->vector) < 10e-7) {
            gsl_vector_set(p1->vector,0,p1->coordinate[0]-p0->coordinate[0]);
            gsl_vector_set(p1->vector,1,p1->coordinate[1]-p0->coordinate[1]);
            gsl_vector_set(p1->vector,2,p1->coordinate[2]-p0->coordinate[2]);
            norm = gsl_blas_dnrm2(p1->vector);
            gsl_vector_scale(p1->vector,(1/norm));
        }
    }

}

//Calculate parameter theta,phi,rho,sigma of each point which are defined in Hobby's Algorithm
//http://link.springer.com/article/10.1007%2FBF02187690
void calculateRhoSigma(waypoint **wayp){
    waypoint *p0    = *wayp;
    double a        = 1.597;
    double b        = 0.07;
    double c        = 0.37;
    double st       = sin(p0->theta);
    double ct       = cos(p0->theta);
    double sp       = sin(p0->phi);
    double cp       = cos(p0->phi);
    double alpha    = a*(st-b*sp)*(sp-b*st)*(ct-cp);

    p0->rho         = (2+alpha)/(1+(1-c)*ct+c*cp);
    p0->sigma       = (2-alpha)/(1+(1-c)*cp+c*ct);
}

//Calculate control points between two points according to Hobby's Algorithm
//There are two control points C1 and C2 between two points P1 and P2,
//the coordinate of C1, C2 are stored in P1->firstControl and P1->secondControl as an array
void calculateControlPoints(waypoint **wayp) {
    waypoint *p0 = *wayp;
    waypoint *p1 = p0->next;

    gsl_vector *pointVector = gsl_vector_alloc(3);
    gsl_vector_set (pointVector,0,p1->coordinate[0]-p0->coordinate[0]);
    gsl_vector_set (pointVector,1,p1->coordinate[1]-p0->coordinate[1]);
    gsl_vector_set (pointVector,2,p1->coordinate[2]-p0->coordinate[2]);

    double norm = gsl_blas_dnrm2(pointVector);

    p0->firstControl->coordinate[0] = p0->coordinate[0]
       + (p0->rho)*norm*gsl_vector_get(p0->vector,0)/(3*(p0->tensionIn));
    p0->firstControl->coordinate[1] = p0->coordinate[1]
       + (p0->rho)*norm*gsl_vector_get(p0->vector,1)/(3*(p0->tensionIn));
    p0->firstControl->coordinate[2] = p0->coordinate[2]
       + (p0->rho)*norm*gsl_vector_get(p0->vector,2)/(3*(p0->tensionIn));

    p0->secondControl->coordinate[0] = p1->coordinate[0]
       - (p0->sigma)*norm*gsl_vector_get(p1->vector,0)/(3*(p1->tensionOut));
    p0->secondControl->coordinate[1] = p1->coordinate[1]
       - (p0->sigma)*norm*gsl_vector_get(p1->vector,1)/(3*(p1->tensionOut));
    p0->secondControl->coordinate[2] = p1->coordinate[2]
       - (p0->sigma)*norm*gsl_vector_get(p1->vector,2)/(3*(p1->tensionOut));
}

//Calculate the direction vector and control points
void calculateParameter(waypoint **wayp) {
    waypoint *p0 = *wayp; //the first node of linked list
    waypoint *p1 = p0->next;

    //There is no parameter for the last point since it doesn't go anywhere
    while(p0->next != NULL) {
        p0->theta = atan2(gsl_vector_get(p0->vector,1),gsl_vector_get(p0->vector,0))
            - atan2((p1->coordinate[1]-p0->coordinate[1]),(p1->coordinate[0]-p0->coordinate[0]));

        p0->phi = atan2((p1->coordinate[1]-p0->coordinate[1]),(p1->coordinate[0]-p0->coordinate[0]))
            - atan2(gsl_vector_get(p1->vector,1),gsl_vector_get(p1->vector,0));

        calculateRhoSigma (&p0);        //Calculate the rho and sigma of current point
        calculateControlPoints (&p0);   //Calculate control points' coordinates of current poinst
        p0 = p1;
        p1 = p1->next;
    }
}

//Do path planning according to control points between each pair of points
void pathPlanning(waypoint** wayp){
//TODO:
//1. de casteljau algorithm
//2. find next point when given current one //either data structure or mathmatic
//3. embedded into existing navigation system
}

int main(int argc, char* argv[]) {
    //Input evaluation
    if (argc == 1){
        printf("No Input\n");
        return -1;
    }
    else if (argc > 3) {
        printf("Too many arguments\n");
        return -1;
    }

    //Store points from command arguments
    waypoint *p0 = addWaypoint(argv[1]);
    waypoint *pCopy = p0;

    int counter = pointsArrangement(&pCopy);
    if (counter < 3) {
        printf ("Too few points\n");
        return -1;
    }
    pCopy = p0;

    //Store customized vector iff necessary
    if (argc == 3) {
        //evaluate # of points and vectors
        if(getVector(argv[2], &pCopy)!=counter) {
            printf ("# of Points and # of Vecotor do not match\n");
            return -1;
        }
        pCopy = p0;
    }

    pCopy = p0;
    //Calaulate vectors if no default
    calculateVector(&pCopy);
    pCopy = p0;

    //Calaulate parameters
    calculateParameter(&pCopy);
    pCopy = p0;

    //Print out results
    int i = 1;
    while (pCopy -> next != NULL) {
        printf("The control points between the %dth point (%f,%f,%f) and the %dth point (%f,%f,%f) are (%f,%f,%f) (%f,%f,%f)\n"
                , i  ,pCopy->coordinate[0],pCopy->coordinate[1],pCopy->coordinate[2]
                , (cycle&&i+1==counter)?1:i+1,pCopy->next->coordinate[0],pCopy->next->coordinate[1],pCopy->next->coordinate[2]
                , pCopy->firstControl ->coordinate[0],pCopy->firstControl ->coordinate[1],pCopy->firstControl ->coordinate[2]
                , pCopy->secondControl->coordinate[0],pCopy->secondControl->coordinate[1],pCopy->secondControl->coordinate[2]
                );
        i ++;
        pCopy = pCopy -> next;

    }
    pCopy = p0;
    pathPlanning(&pCopy);
    return 0;
}
