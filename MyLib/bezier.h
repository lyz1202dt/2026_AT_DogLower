#ifndef __CORE_H__
#define __CORE_H__

#include <stdio.h>
#include <math.h>

//#ifndef M_PI
#define M_PI 3.141592653
//#endif

typedef struct
{
    float p1_x;
    float p1_y;
    float p2_x;
    float p2_y;
}BezierLine;

float bezier_get_t(float x,float p1_x,float p2_x);
float bezier_get_y(float t,float x,float p1_y,float p2_y);

#define BezierTransform(x,bezier) bezier_get_y(bezier_get_t(x,bezier.p1_x,bezier.p2_x),x,bezier.p1_y,bezier.p2_y)

#endif