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
    vec2D contactPoints[2];
    float penetrations[2];  
    int contactCount = 0;
    float normalImpulse[2] = {0.0f, 0.0f};
    float tangentImpulse[2] = {0.0f, 0.0f};
};

vec2D distanceVec(const vec2D& a, const vec2D& b){ 
    return vec2D(a.x - b.x, a.y - b.y);
}

inline std::array<vec2D, 4> getOBBVertices(const box& b, const vec2D& pos, float angle) {
    std::array<vec2D, 4> vertices;
    float hw = b.width / 2.0f;
    float hh = b.height / 2.0f;
    float c = std::cos(angle);
    float s = std::sin(angle);

    vec2D corners[4] = {
        vec2D(-hw, -hh), vec2D(hw, -hh),
        vec2D(hw, hh), vec2D(-hw, hh)
    };

    for(int i = 0; i < 4; i++) {
        vertices[i].x = pos.x + (corners[i].x * c - corners[i].y * s);
        vertices[i].y = pos.y + (corners[i].x * s + corners[i].y * c);
    }
    return vertices;
}

struct FaceInfo {
    int i0, i1;      // vertex indices forming the face (i0 -> i1)
    vec2D normal;    // outward unit normal of that face
};

// Finds the edge of `verts` whose outward normal points most closely along `dir`.
// Works for any CCW-wound quad, so it's robust to rotation — no reliance on
// local-axis bookkeeping like your old axes[] array.
inline FaceInfo findBestFace(const std::array<vec2D, 4>& verts, const vec2D& dir) {
    float bestDot = -1e9f;
    int bestIdx = 0;
    vec2D bestNormal;

    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        vec2D edge = verts[j] - verts[i];
        vec2D normal = vec2D(edge.y, -edge.x).normalize(); // outward for CCW winding

        float d = normal.dot(dir);
        if (d > bestDot) {
            bestDot = d;
            bestIdx = i;
            bestNormal = normal;
        }
    }
    return { bestIdx, (bestIdx + 1) % 4, bestNormal };
}

// Clips segment [v0,v1] against one half-plane: keeps points where
// dot(planeDir, p) <= offset, interpolating the cut point if the segment
// straddles the plane. Returns how many points were written (0–2).
inline int clipSegmentToLine(const vec2D& v0, const vec2D& v1, const vec2D& planeDir, float offset, vec2D out[2]) {
    int count = 0;
    float d0 = planeDir.dot(v0) - offset;
    float d1 = planeDir.dot(v1) - offset;

    if (d0 <= 0.0f) out[count++] = v0;
    if (d1 <= 0.0f) out[count++] = v1;

    if (d0 * d1 < 0.0f) { // straddles — compute intersection
        float t = d0 / (d0 - d1);
        out[count++] = v0 + (v1 - v0) * t;
    }
    return count;
}

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
            // Populate array instead of single point
            m.contactPoints[0] = vec2D(posA.x + m.normal.x * circA.radius, posA.y + m.normal.y * circA.radius);
            m.contactCount = 1; 
            return m;
        }
    }
    
    // ---------------------------------------------------------
    // CIRCLE vs BOX 
    // ---------------------------------------------------------
    else if (a.type == shapeType::CIRCLE && b.type == shapeType::BOX) {
        const circle& circA = static_cast<const circle&>(a);
        const box& boxB = static_cast<const box&>(b);

        manifold m; 
        
        vec2D diff = distanceVec(posA, posB); 
        float c = std::cos(-angleB);
        float s = std::sin(-angleB);
        vec2D localDiff = vec2D(diff.x * c - diff.y * s, diff.x * s + diff.y * c);

        float halfWidth = boxB.width / 2.0f;
        float halfHeight = boxB.height / 2.0f;
        vec2D localClamped;
        localClamped.x = std::clamp(localDiff.x, -halfWidth, halfWidth);
        localClamped.y = std::clamp(localDiff.y, -halfHeight, halfHeight);

        float cInv = std::cos(angleB);
        float sInv = std::sin(angleB);
        vec2D closestPoint = vec2D(
            posB.x + (localClamped.x * cInv - localClamped.y * sInv),
            posB.y + (localClamped.x * sInv + localClamped.y * cInv)
        );

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
            // Populate array
            m.contactPoints[0] = closestPoint;
            m.contactCount = 1;
        }
        return m;
    }

    // ---------------------------------------------------------
    // BOX vs CIRCLE
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

            if (maxA < minB || maxB < minA) return m; 

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            
            if (overlap < minOverlap) {
                minOverlap = overlap;
                minAxis = axis;
            }
        }

        m.isColliding = true;
        m.penetration = minOverlap;
        
        vec2D diff = distanceVec(posB, posA);
        if (diff.x * minAxis.x + diff.y * minAxis.y < 0) {
            minAxis = vec2D(-minAxis.x, -minAxis.y);
        }
        m.normal = minAxis;


        // === STRUCTURAL INCIDENT EDGE DETECTION (The "Segment" Approach) ===
        float dot0 = std::abs(m.normal.x * axes[0].x + m.normal.y * axes[0].y);
        float dot1 = std::abs(m.normal.x * axes[1].x + m.normal.y * axes[1].y);
        bool normalIsFromA = (dot0 > 0.99f || dot1 > 0.99f);

        vec2D refNormal = normalIsFromA ? m.normal : vec2D(-m.normal.x, -m.normal.y);
        vec2D searchDir = vec2D(-refNormal.x, -refNormal.y);

        const std::array<vec2D, 4>& refVerts = normalIsFromA ? vertsA : vertsB;
        const std::array<vec2D, 4>& incVerts  = normalIsFromA ? vertsB : vertsA;

        FaceInfo refFace = findBestFace(refVerts, refNormal);
        FaceInfo incFace = findBestFace(incVerts, searchDir);

        vec2D refP1 = refVerts[refFace.i0];
        vec2D refP2 = refVerts[refFace.i1];
        vec2D incP1 = incVerts[incFace.i0];
        vec2D incP2 = incVerts[incFace.i1];

        vec2D tangent = (refP2 - refP1).normalize();

        // Clip incident edge against the two side planes of the reference face
        vec2D clip1[2];
        int n1 = clipSegmentToLine(incP1, incP2, vec2D(-tangent.x, -tangent.y), -tangent.dot(refP1), clip1);
        if (n1 < 2) { clip1[0] = incP1; clip1[1] = incP2; n1 = 2; } // geometric safety net

        vec2D clip2[2];
        int n2 = clipSegmentToLine(clip1[0], clip1[1], tangent, tangent.dot(refP2), clip2);

        // Per-point depth check — only keep points that are actually penetrating
        const float SLOP = 0.01f * std::min(std::min(boxA.width, boxA.height), std::min(boxB.width, boxB.height));

        m.contactCount = 0;
        for (int i = 0; i < n2 && m.contactCount < 2; i++) {
            float separation = refFace.normal.dot(clip2[i] - refP1); // >0 outside, <0 penetrating
            float depth = -separation;

            if (depth > -SLOP) {
                m.contactPoints[m.contactCount] = clip2[i];
                m.penetrations[m.contactCount] = std::max(depth, 0.0f);
                m.contactCount++;
            }
        }

        if (m.contactCount == 0) m.isColliding = false;
        return m;
    }
    return manifold();
}