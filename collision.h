#include "vector2D.h"
#include "shape.h"
#include "circle.h"
#pragma once

vec2D distanceVec(const vec2D& a, const vec2D& b){
    return vec2D(a.x - b.x, a.y - b.y);
}

bool isCollided(const shape& a, const shape& b, 
    const vec2D& posA, const vec2D& posB){
        if(a.type == shapeType::CIRCLE && b.type == shapeType::CIRCLE){
            const circle& circA = static_cast<const circle&>(a);
            const circle& circB = static_cast<const circle&>(b);
            vec2D distVec = distanceVec(posA, posB);
            float dist = distVec.magnitude();
            return dist < (circA.radius + circB.radius);
        }
    
}

