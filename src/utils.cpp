#include "utils.h"

float lerp(float a, float b, float w){
	return a*w + b*(1-w);
}

float d2r(float d){
	return d*M_PI/180;
}

float r2d(float r){
	return r * 180 / M_PI;
}

float posr(float x){
	float TWO_PI = 2*M_PI;
    x = fmod(x,TWO_PI);
    if (x < 0)
        x += TWO_PI;
    return x;
}
