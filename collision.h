#include "vector2D.h"
#include "shape.h"
#include "circle.h"
#pragma once

struct manifold {
    bool isColliding = false;
    vec2D normal;
    float penetration; //The total overlap of 2 objects
};

vec2D distanceVec(const vec2D& a, const vec2D& b){
    return vec2D(a.x - b.x, a.y - b.y);
}

manifold generateManifold(const shape& a, const shape& b, 
    const vec2D& posA, const vec2D& posB){
        if(a.type == shapeType::CIRCLE && b.type == shapeType::CIRCLE){
            const circle& circA = static_cast<const circle&>(a);
            const circle& circB = static_cast<const circle&>(b);
            vec2D distVec = distanceVec(posA, posB);
            float dist = distVec.magnitude();
            float radiusSum = circA.radius + circB.radius;

            if(dist < radiusSum){
                manifold m;
                m.isColliding = true;
                m.penetration = radiusSum - dist;
                if(dist != 0){
                    m.normal = distVec.normalize();
                } else {
                    m.normal = vec2D(1, 0);
                    m.penetration = circA.radius;
                }
                return m;

            }
        }
    
}

