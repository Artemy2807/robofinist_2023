#ifndef __MATH_FN
#define __MATH_FN

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>
#define PI 					(float)3.141592654

#define ABS(x) 			((x) > 0 ? (x) : -(x))
#define RAD(x)			(((x) * PI) / 180.0f)
#define min(a,b) 		(((a) < (b)) ? (a) : (b))
#define max(a,b) 		(((a) > (b)) ? (a) : (b))
#define abs(a)			(((a) < 0) ? (-(a)) : (a))
#define sign(x)			(((x) < 0) ? (-1) : (1))

float constrain_angle(float x);

float euclidian_distance_nonsqrt(float x, float y);

float euclidian_distance(float x, float y);

#ifdef __cplusplus
}
#endif

#endif
