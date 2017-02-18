#ifndef __UTILS_H__
#define __UTILS_H__

#include <cmath>

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

extern float lerp(float a, float b, float w);
extern float d2r(float d);
extern float r2d(float r);
extern float posr(float r);

#endif
