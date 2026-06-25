#include "World.h"
#include "collision.h"
#include <algorithm>
#include <vector>
#include <cmath> // REQUIRED for std::abs, std::sqrt

World::World(vec2D gravity) {
    this->gravity = gravity;
}

void World::addBody(RigidBody* body) {
    bodies.push_back(body);
}

inline float cross2D(const vec2D& a, const vec2D& b) {
    return a.x * b.y - a.y * b.x;
}

bool AABBOverlapCheck(RigidBody* a, RigidBody* b) {
    return true; 
}

// Struct to hold collisions cleanly for the iterative solver
struct CollisionData {
    RigidBody* bodyA;
    RigidBody* bodyB;
    manifold m;
    float bounce[2]; // Stores pre-calculated restitution for up to 2 contact points
};

void World::step(float dt) {
    debugLines.clear();
    
    // STAGE I: INTEGRATION
    for(RigidBody* body: bodies) {
        body->applyForce(vec2D(gravity.x * body->getMass(), gravity.y * body->getMass()));
        
        // Directional vectors for testing (Acceleration/Forces - Orange)
        // drawLine(body->position, body->position + (body->acceleration * 0.2f), sf::Color(255, 165, 0));
        
        body->update(dt);
        
        // Directional vectors for testing (Velocity - Green)
        // drawLine(body->position, body->position + (body->velocity * 0.3f), sf::Color::Green);
    }

    // STAGE II: MANIFOLD GENERATION
    std::vector<CollisionData> activeCollisions;
    
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];

            if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) {
                continue; 
            }

            if (AABBOverlapCheck(bodyA, bodyB)) {
                manifold m = generateManifold(*(bodyA->m_shape), *(bodyB->m_shape), bodyA->position, bodyB->position, bodyA->angle, bodyB->angle);
                
                if (m.isColliding && m.contactCount > 0) {
                    CollisionData col;
                    col.bodyA = bodyA;
                    col.bodyB = bodyB;
                    col.m = m;
                    activeCollisions.push_back(col);
                }
            }
        }
    }

    // Directional vectors for testing (Contact points and Normals)
    /*
    for (auto& col : activeCollisions) {
        manifold& m = col.m; 
        for (int k = 0; k < m.contactCount; k++) {
            vec2D contact = m.contactPoints[k];
            
            // Draw Contact Point (Red crosshair)
            drawLine(contact - vec2D(8,0), contact + vec2D(8,0), sf::Color::Red);
            drawLine(contact - vec2D(0,8), contact + vec2D(0,8), sf::Color::Red);
            
            // Draw Collision Normal (Yellow)
            drawLine(contact, contact + (m.normal * 40.0f), sf::Color::Yellow);
        }
    }
    */

    // Initialize accumulators and calculate restitution bias
    for (auto& col : activeCollisions) {
        RigidBody* bodyA = col.bodyA;
        RigidBody* bodyB = col.bodyB;
        manifold& m = col.m;
        
        for (int k = 0; k < m.contactCount; k++) {
            // 1. Zero out uninitialized memory for intra-frame accumulators
            m.normalImpulse[k] = 0.0f;
            m.tangentImpulse[k] = 0.0f;

            // 2. Pre-calculate restitution bias using the INITIAL impact velocity
            vec2D rA = m.contactPoints[k] - bodyA->position;
            vec2D rB = m.contactPoints[k] - bodyB->position;
            vec2D vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
            vec2D vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);
            
            float initialVelocityAlongNormal = (vB_contact - vA_contact).dot(m.normal);
            
            float restitution = std::min(bodyA->restitution, bodyB->restitution);
            if (std::abs(initialVelocityAlongNormal) < 25.0f) {
                restitution = 0.0f; // Slop threshold to stop micro-bouncing on resting objects
            }
            
            // Store the bounce target velocity
            col.bounce[k] = restitution * initialVelocityAlongNormal;
        }
    }

    // STAGE III: THE ITERATIVE SOLVER
    const int ITERATIONS = 30;

    for (int it = 0; it < ITERATIONS; it++) {
        for (auto& col : activeCollisions) {
            RigidBody* bodyA = col.bodyA;
            RigidBody* bodyB = col.bodyB;
            manifold& m = col.m;

            float invMassA = bodyA->getInvMass();
            float invMassB = bodyB->getInvMass();
            float invInertiaA = bodyA->getInvInertia();
            float invInertiaB = bodyB->getInvInertia();

            if (invMassA + invMassB == 0.0f) continue;

            for (int k = 0; k < m.contactCount; k++) {
                vec2D rA = m.contactPoints[k] - bodyA->position;
                vec2D rB = m.contactPoints[k] - bodyB->position;

                // NORMAL IMPULSE
                vec2D vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
                vec2D vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);
                vec2D relativeVelocity = vB_contact - vA_contact;
                
                float velocityAlongNormal = relativeVelocity.dot(m.normal);

                float rA_cross_N = cross2D(rA, m.normal);
                float rB_cross_N = cross2D(rB, m.normal);

                float invMassSum = invMassA + invMassB + 
                    (rA_cross_N * rA_cross_N * invInertiaA) + 
                    (rB_cross_N * rB_cross_N * invInertiaB);

                // Use the pre-calculated bounce bias instead of checking restitution dynamically
                float j = -(velocityAlongNormal + col.bounce[k]) / invMassSum;

                float oldNormalImpulse = m.normalImpulse[k];
                m.normalImpulse[k] = std::max(oldNormalImpulse + j, 0.0f);
                float delta_j = m.normalImpulse[k] - oldNormalImpulse;
                vec2D normalPush = m.normal * delta_j;

                bodyA->velocity = bodyA->velocity - (normalPush * invMassA);
                bodyA->angularVelocity -= cross2D(rA, normalPush) * invInertiaA;
                bodyB->velocity = bodyB->velocity + (normalPush * invMassB);
                bodyB->angularVelocity += cross2D(rB, normalPush) * invInertiaB;

                // TANGENTIAL IMPULSE (Friction)
                vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
                vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);
                relativeVelocity = vB_contact - vA_contact;

                vec2D tangent = relativeVelocity - (m.normal * relativeVelocity.dot(m.normal));
                
                if (tangent.magnitude() > 0.0001f) {
                    tangent = tangent.normalize();
                    float velocityAlongTangent = relativeVelocity.dot(tangent);

                    float rA_cross_T = cross2D(rA, tangent);
                    float rB_cross_T = cross2D(rB, tangent);

                    float invMassSumTangent = invMassA + invMassB + 
                        (rA_cross_T * rA_cross_T * invInertiaA) + 
                        (rB_cross_T * rB_cross_T * invInertiaB);

                    float jt = -velocityAlongTangent / invMassSumTangent;
                    
                    float mu = std::sqrt(bodyA->dynamicFriction * bodyA->dynamicFriction + bodyB->dynamicFriction * bodyB->dynamicFriction);
                    float baselineGravityImpulse = 3.2f / (invMassA + invMassB);
                    float maxFriction = (m.normalImpulse[k] + baselineGravityImpulse) * mu;

                    float oldTangentImpulse = m.tangentImpulse[k];
                    
                    // Fixed clamp to compile correctly across all standards
                    m.tangentImpulse[k] = std::max(-maxFriction, std::min(oldTangentImpulse + jt, maxFriction));
                    
                    float delta_jt = m.tangentImpulse[k] - oldTangentImpulse;
                    vec2D frictionPush = tangent * delta_jt;

                    bodyA->velocity = bodyA->velocity - (frictionPush * invMassA);
                    bodyA->angularVelocity -= cross2D(rA, frictionPush) * invInertiaA;
                    bodyB->velocity = bodyB->velocity + (frictionPush * invMassB);
                    bodyB->angularVelocity += cross2D(rB, frictionPush) * invInertiaB;
                }
            }
        }
    }

    // STAGE IV: POSITIONAL CORRECTION
    for (auto& col : activeCollisions) {
        RigidBody* bodyA = col.bodyA;
        RigidBody* bodyB = col.bodyB;
        manifold& m = col.m;

        float invMassA = bodyA->getInvMass();
        float invMassB = bodyB->getInvMass();
        float totalInvMass = invMassA + invMassB;

        if (totalInvMass == 0.0f) continue;

        const float percent = 0.85f; 
        const float slop = 0.15f;   
        
        float penetration = std::max(m.penetration - slop, 0.0f);
        vec2D correction = m.normal * ((penetration * percent) / totalInvMass);

        bodyA->position = bodyA->position - (correction * invMassA);
        bodyB->position = bodyB->position + (correction * invMassB);
    }

    // STAGE V: CLEANUP
    for(RigidBody* body: bodies) {
        body->clearForces();
    }
}