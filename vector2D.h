#pragma once 
#include <math.h>

class vec2D{
    public:
    float x,y;
    
    vec2D();
    vec2D (float x, float y);

    vec2D operator+(const vec2D& a)const;
    vec2D operator-(const vec2D& a)const;
    vec2D operator*(float scalar) const;

    float magnitude() const;
    vec2D normalize() const;
    float dot(const vec2D& a) const;
};