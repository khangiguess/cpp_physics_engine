#include "vector2D.h"
#include "shape.h"
#include "circle.h"
#include "box.h"
#include <algorithm>
#include <cmath>
#include <array> 
#pragma once

struct manifold {
    bool isColliding = false;
    vec2D normal;
    float penetration;
    vec2D contactPoint;
};

vec2D distanceVec(const vec2D& a, const vec2D& b){ // points from b to a
    return vec2D(a.x - b.x, a.y - b.y);
}

// Helper function to calculate the 4 rotated corners of an OBB in world space
inline std::array<vec2D, 4> getOBBVertices(const box& b, const vec2D& pos, float angle) {
    std::array<vec2D, 4> vertices;
    float hw = b.width / 2.0f;
    float hh = b.height / 2.0f;
    float c = std::cos(angle);
    float s = std::sin(angle);

    // Unrotated local corners
    vec2D corners[4] = {
        vec2D(-hw, -hh), vec2D(hw, -hh),
        vec2D(hw, hh), vec2D(-hw, hh)
    };

    // Rotate and translate to world space
    for(int i = 0; i < 4; i++) {
        vertices[i].x = pos.x + (corners[i].x * c - corners[i].y * s);
        vertices[i].y = pos.y + (corners[i].x * s + corners[i].y * c);
    }
    return vertices;
}

// UPDATED: Now accepts angleA and angleB to support rotation!
manifold generateManifold(const shape& a, const shape& b, 
    const vec2D& posA, const vec2D& posB, float angleA = 0.0f, float angleB = 0.0f){
        
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
            m.contactPoint = vec2D(posA.x + m.normal.x * circA.radius, posA.y + m.normal.y * circA.radius);
            return m;
        }
    }
    
    // ---------------------------------------------------------
    // CIRCLE vs BOX (Rotated OBB)
    // ---------------------------------------------------------
    else if (a.type == shapeType::CIRCLE && b.type == shapeType::BOX) {
        const circle& circA = static_cast<const circle&>(a);
        const box& boxB = static_cast<const box&>(b);

        manifold m; 
        
        // 1. Convert Circle center to Box's local (unrotated) space
        vec2D diff = distanceVec(posA, posB); // Points from Box to Circle
        float c = std::cos(-angleB);
        float s = std::sin(-angleB);
        vec2D localDiff = vec2D(diff.x * c - diff.y * s, diff.x * s + diff.y * c);

        // 2. Clamp in local space
        float halfWidth = boxB.width / 2.0f;
        float halfHeight = boxB.height / 2.0f;
        vec2D localClamped;
        localClamped.x = std::clamp(localDiff.x, -halfWidth, halfWidth);
        localClamped.y = std::clamp(localDiff.y, -halfHeight, halfHeight);

        // 3. Convert Clamped point back to World Space
        float cInv = std::cos(angleB);
        float sInv = std::sin(angleB);
        vec2D closestPoint = vec2D(
            posB.x + (localClamped.x * cInv - localClamped.y * sInv),
            posB.y + (localClamped.x * sInv + localClamped.y * cInv)
        );

        // 4. Test distance
        vec2D distanceToClosest = distanceVec(closestPoint, posA);
        float dist = distanceToClosest.magnitude();

        if (dist < circA.radius) {
            m.isColliding = true;
            m.penetration = circA.radius - dist;
            
            if (dist == 0.0f) {
                m.normal = vec2D(0, -1); 
            } else {
                m.normal = distanceToClosest.normalize();
            }
            m.contactPoint = closestPoint;
        }
        return m;
    }

    // ---------------------------------------------------------
    // BOX vs CIRCLE (Swap arguments & angles)
    // ---------------------------------------------------------
    else if (a.type == shapeType::BOX && b.type == shapeType::CIRCLE) {
        manifold m = generateManifold(b, a, posB, posA, angleB, angleA);
        
        if (m.isColliding) {
            m.normal = vec2D(-m.normal.x, -m.normal.y); 
        }
        return m;
    }

    // ---------------------------------------------------------
    // BOX vs BOX (Oriented Bounding Boxes using SAT)
    // ---------------------------------------------------------
    else if (a.type == shapeType::BOX && b.type == shapeType::BOX) {
        const box& boxA = static_cast<const box&>(a);
        const box& boxB = static_cast<const box&>(b);

        manifold m;

        // The 4 local axes (normals) of the two boxes
        vec2D axes[4] = {
            vec2D(std::cos(angleA), std::sin(angleA)),
            vec2D(-std::sin(angleA), std::cos(angleA)),
            vec2D(std::cos(angleB), std::sin(angleB)),
            vec2D(-std::sin(angleB), std::cos(angleB))
        };

        auto vertsA = getOBBVertices(boxA, posA, angleA);
        auto vertsB = getOBBVertices(boxB, posB, angleB);

        float minOverlap = 1000000.0f;
        vec2D minAxis;

        // Project all corners onto all 4 axes to check for gaps
        for(int i = 0; i < 4; i++) {
            vec2D axis = axes[i];
            
            float minA = 1000000.0f, maxA = -1000000.0f;
            for(const auto& v : vertsA) {
                float p = v.x * axis.x + v.y * axis.y;
                minA = std::min(minA, p);
                maxA = std::max(maxA, p);
            }
            
            float minB = 1000000.0f, maxB = -1000000.0f;
            for(const auto& v : vertsB) {
                float p = v.x * axis.x + v.y * axis.y;
                minB = std::min(minB, p);
                maxB = std::max(maxB, p);
            }

            // If we found a gap, SAT guarantees they are NOT colliding!
            if (maxA < minB || maxB < minA) {
                return m; 
            }

            // Calculate overlap length
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            
            // Keep track of the axis with the smallest overlap
            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }

        // Overlap occurred on ALL axes, so they are colliding!
        m.isColliding = true;
        m.penetration = minOverlap;
        
        // Ensure the normal always points from A to B
        vec2D diff = distanceVec(posB, posA);
        if (diff.x * minAxis.x + diff.y * minAxis.y < 0) {
            minAxis = vec2D(-minAxis.x, -minAxis.y);
        }
        m.normal = minAxis;

        // Contact Point Approximation: 
        // We find the corner of the incident box that is pushing deepest against the reference box.
        float dot0 = std::abs(m.normal.x * axes[0].x + m.normal.y * axes[0].y);
        float dot1 = std::abs(m.normal.x * axes[1].x + m.normal.y * axes[1].y);
        bool normalIsFromA = (dot0 > 0.99f || dot1 > 0.99f);

        vec2D contactPoint;
        if (normalIsFromA) {
            float minProj = 1000000.0f;
            for (const auto& v : vertsB) {
                float proj = v.x * m.normal.x + v.y * m.normal.y;
                if (proj < minProj) { minProj = proj; contactPoint = v; }
            }
        } else {
            float maxProj = -1000000.0f;
            for (const auto& v : vertsA) {
                float proj = v.x * m.normal.x + v.y * m.normal.y;
                if (proj > maxProj) { maxProj = proj; contactPoint = v; }
            }
        }
        m.contactPoint = contactPoint;

        return m;
    }

    return manifold();
}