#include <iostream>
#include <string>
#include <math.h>
using namespace std;

struct vec2{
    float x,y;
    vec2 (float x=0, float y=0);
    vec2 (float x, float y){
        this -> x = x;
        this -> y = y;
    }

    void operator+(){}
    void operator-(){}
    void operator*(){}
};
