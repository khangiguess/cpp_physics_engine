#include <iostream>
#include <string>
#include <math.h>
using namespace std;

class vec2D{
    public:
    float x,y;
    
    vec2D();
    vec2D (float x=0, float y=0);
    vec2D (float x, float y){
        this -> x = x;
        this -> y = y;
    }

    void operator+(){}
    void operator-(){}
    void operator*(){}


    float magnitude() const;
    vec2D normalize() const;
    float dot(const vec2D a) const;
    float cross(const vec2D a) const;
};