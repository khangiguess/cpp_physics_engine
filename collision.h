#include "vector2D.h"
#include "shape.h"
#include "circle.h"
#include "box.h"
#include <algorithm>
#include <cmath> // Added for std::abs and std::max/min
#pragma once

struct manifold {
    bool isColliding = false;
    vec2D normal;
    float penetration; //The total overlap of 2 objects
    vec2D contactPoint; //The physical location of the collision strike in world space
};

vec2D distanceVec(const vec2D& a, const vec2D& b){ //points from a to b
    return vec2D(a.x - b.x, a.y - b.y);
}

manifold generateManifold(const shape& a, const shape& b, 
    const vec2D& posA, const vec2D& posB){
        
        // ---------------------------------------------------------
        // CIRCLE vs CIRCLE
        // ---------------------------------------------------------
        if(a.type == shapeType::CIRCLE && b.type == shapeType::CIRCLE){
            const circle& circA = static_cast<const circle&>(a);
            const circle& circB = static_cast<const circle&>(b);
            vec2D distVec = distanceVec(posB, posA);
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
                
                // Contact point is exactly on the edge of Circle A along the normal vector
                m.contactPoint = vec2D(posA.x + m.normal.x * circA.radius, posA.y + m.normal.y * circA.radius);
                return m;
            }
        }
    
        // ---------------------------------------------------------
        // CIRCLE vs BOX
        // ---------------------------------------------------------
        else if (a.type == shapeType::CIRCLE && b.type == shapeType::BOX) {
            const circle& circA = static_cast<const circle&>(a);
            const box& boxB = static_cast<const box&>(b);

            manifold m; // Defaults to false

            // 1. Get vector pointing from Box center to Circle center
            vec2D diff = distanceVec(posA, posB); 

            // 2. Calculate the half-extents (half width, half height) of the box
            float halfWidth = boxB.width / 2.0f;
            float halfHeight = boxB.height / 2.0f;

            // 3. Clamp the vector to find the Closest Point on the Box's surface
            vec2D clamped;
            clamped.x = std::clamp(diff.x, -halfWidth, halfWidth);
            clamped.y = std::clamp(diff.y, -halfHeight, halfHeight);

            // Calculate the absolute world position of that Closest Point
            vec2D closestPoint = vec2D(posB.x + clamped.x, posB.y + clamped.y);

            // 4. Get the vector from the Closest Point to the Circle Center
            vec2D distanceToClosest = distanceVec(closestPoint, posA);
            float dist = distanceToClosest.magnitude();

            // 5. If distance is less than radius, we have a collision!
            if (dist < circA.radius) {
                m.isColliding = true;
                m.penetration = circA.radius - dist;
                
                //if the circle's center perfectly overlaps the closest point
                if (dist == 0.0f) {
                    // Force a normal pointing upwards to push it out
                    m.normal = vec2D(0, -1); 
                } else {
                    m.normal = distanceToClosest.normalize();
                }
                
                // Contact point is exactly the closest point we clamped to on the Box's surface!
                m.contactPoint = closestPoint;
            }
            return m;
    }

    // ---------------------------------------------------------
    // BOX vs CIRCLE
    // ---------------------------------------------------------
    //Swap the argument in case parameters passed in as BOX-CIRCLE
    else if (a.type == shapeType::BOX && b.type == shapeType::CIRCLE) {
        manifold m = generateManifold(b, a, posB, posA);
        
        if (m.isColliding) {
            m.normal = vec2D(-m.normal.x, -m.normal.y); 
        }
        // The contact point returned by the swapped call is already in correct world space, no change needed!
        return m;
    }

    // ---------------------------------------------------------
    // BOX vs BOX
    // ---------------------------------------------------------
    else if (a.type == shapeType::BOX && b.type == shapeType::BOX) {
        const box& boxA = static_cast<const box&>(a);
        const box& boxB = static_cast<const box&>(b);

        manifold m;

        vec2D diff = distanceVec(posB, posA);

        //overlap along the X-axis
        float halfWidthA = boxA.width / 2.0f;
        float halfWidthB = boxB.width / 2.0f;
        float overlapX = (halfWidthA + halfWidthB) - std::abs(diff.x);

        // No overlap along X means they are separated
        if (overlapX <= 0.0f) {
            return m;
        }

        // 3. Calculate overlap along the Y-axis
        float halfHeightA = boxA.height / 2.0f;
        float halfHeightB = boxB.height / 2.0f;
        float overlapY = (halfHeightA + halfHeightB) - std::abs(diff.y);

        // No overlap along Y means they are separated
        if (overlapY <= 0.0f) {
            return m;
        }

        //Overlap detected on both axes
        m.isColliding = true;

        //Resolve along the axis of MINIMUM penetration
        if (overlapX < overlapY) {
            m.penetration = overlapX;
            // Normal points from A to B
            if (diff.x > 0.0f) {
                m.normal = vec2D(1.0f, 0.0f); // B is to the right
            } else {
                m.normal = vec2D(-1.0f, 0.0f); // B is to the left
            }
        } else {
            m.penetration = overlapY;
            // Normal points from A to B
            if (diff.y > 0.0f) {
                m.normal = vec2D(0.0f, 1.0f); // B is below A
            } else {
                m.normal = vec2D(0.0f, -1.0f); // B is above A
            }
        }

        // Contact point for box-vs-box is estimated as the exact geometric center of the overlapping rectangle
        float minX = std::max(posA.x - halfWidthA, posB.x - halfWidthB);
        float maxX = std::min(posA.x + halfWidthA, posB.x + halfWidthB);
        float minY = std::max(posA.y - halfHeightA, posB.y - halfHeightB);
        float maxY = std::min(posA.y + halfHeightA, posB.y + halfHeightB);

        m.contactPoint.x = (minX + maxX) / 2.0f;
        m.contactPoint.y = (minY + maxY) / 2.0f;

        return m;
    }

    // Default: no collision for unsupported shape pairs
    return manifold();
}