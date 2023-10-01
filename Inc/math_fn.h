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

inline float constrain_angle(float x);

#ifdef __cplusplus
}
#endif

#endif
