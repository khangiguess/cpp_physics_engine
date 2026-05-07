#include "vector2D.h"
#include "shape.h"
#include "circle.h"
#pragma once

vec2D distanceVec(const vec2D& a, const vec2D& b){
    return vec2D(a.x - b.x, a.y - b.y);
}

bool isCollided(const shape& a, const shape& b){
    
}

