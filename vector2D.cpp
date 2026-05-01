#include <math.h>
#include "vector2D.h"

vec2D :: vec2D(){
    x = 0;
    y = 0;
}

vec2D :: vec2D(float x, float y){
    this -> x = x;
    this -> y = y;
}

vec2D vec2D::operator+(const vec2D& a)const{
    float sumX, sumY;
    sumX = this -> x + a.x;
    sumY = this -> y + a.y;
    return vec2D(sumX, sumY);
}

vec2D vec2D::operator-(const vec2D& a)const{
    float sumX, sumY;
    sumX = this -> x - a.x;
    sumY = this -> y - a.y;
    return vec2D(sumX, sumY);
}

float vec2D::magnitude() const {
    float   sum = 0, 
            xSquared = (this->x)*(this->x), 
            ySquared = (this->y)*(this->y);
    sum = xSquared + ySquared;
    return sqrt(sum);
}

vec2D vec2D::normalize() const{
    float xVal= this -> x, yVal= this -> y, length = magnitude();

    if(length == 0.0f)

    xVal /= length;
    yVal /= length;
    return vec2D(xVal,yVal);
}

float vec2D::dot(const vec2D& a) const {
    float xProd = (this -> x)*(a.x), yProd = (this -> y)*(a.y);
    return (xProd + yProd);
}