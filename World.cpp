#include "World.h"
#include "collision.h"
#include <algorithm>
#include <vector>

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
};

void World::step(float dt) {
    // =========================================================
    // STAGE I: INTEGRATION
    // =========================================================
    for(RigidBody* body: bodies) {
        body->applyForce(vec2D(gravity.x * body->getMass(), gravity.y * body->getMass()));
        body->update(dt);
    }

    // =========================================================
    // STAGE II: BROAD PHASE & MANIFOLD GENERATION
    // =========================================================
    std::vector<CollisionData> activeCollisions;
    
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];

            // === THE FIX: IGNORE STATIC VS STATIC COLLISIONS ===
            // If both objects have infinite mass, they cannot affect each other.
            if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) {
                continue; 
            }

            if (AABBOverlapCheck(bodyA, bodyB)) {
                manifold m = generateManifold(*(bodyA->m_shape), *(bodyB->m_shape), bodyA->position, bodyB->position, bodyA->angle, bodyB->angle);
                
                if (m.isColliding && m.contactCount > 0) {
                    activeCollisions.push_back({bodyA, bodyB, m});
                }
            }
        }
    }

    // =========================================================
    // STAGE III: THE ITERATIVE SOLVER (NARROW PHASE)
    // =========================================================
    const int ITERATIONS = 10; // Number of passes to stabilize the constraints

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

            // Solve each contact point sequentially
            for (int k = 0; k < m.contactCount; k++) {
                vec2D rA = m.contactPoints[k] - bodyA->position;
                vec2D rB = m.contactPoints[k] - bodyB->position;

                // 1. Calculate Relative Velocity
                vec2D vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
                vec2D vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);
                vec2D relativeVelocity = vB_contact - vA_contact;
                
                float velocityAlongNormal = relativeVelocity.dot(m.normal);

                // 2. Normal Impulse (Bounce)
                if (velocityAlongNormal < 0.0f) {
                    float rA_cross_N = cross2D(rA, m.normal);
                    float rB_cross_N = cross2D(rB, m.normal);

                    float invMassSum = invMassA + invMassB + 
                        (rA_cross_N * rA_cross_N * invInertiaA) + 
                        (rB_cross_N * rB_cross_N * invInertiaB);

                    float restitution = std::min(bodyA->restitution, bodyB->restitution);
                    
                    // CRITICAL FIX: Increased threshold to catch pixel-scale gravity!
                    if (std::abs(velocityAlongNormal) < 25.0f) restitution = 0.0f; 

                    float j = -(1.0f + restitution) * velocityAlongNormal / invMassSum;
                    vec2D impulse = m.normal * j;

                    // Apply immediately so the friction calculation sees the result!
                    bodyA->velocity = bodyA->velocity - (impulse * invMassA);
                    bodyA->angularVelocity -= cross2D(rA, impulse) * invInertiaA;
                    bodyB->velocity = bodyB->velocity + (impulse * invMassB);
                    bodyB->angularVelocity += cross2D(rB, impulse) * invInertiaB;

                    // 3. Friction Impulse (Unconditional - Hack Removed)
                    // Recalculate velocity after normal impulse
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
                        
                        float mu_s = std::sqrt(bodyA->staticFriction * bodyA->staticFriction + bodyB->staticFriction * bodyB->staticFriction);
                        float mu_d = std::sqrt(bodyA->dynamicFriction * bodyA->dynamicFriction + bodyB->dynamicFriction * bodyB->dynamicFriction);

                        float jt_applied = 0.0f;
                        if (std::abs(jt) < j * mu_s) jt_applied = jt;
                        else jt_applied = j * mu_d * (jt > 0.0f ? 1.0f : -1.0f);

                        vec2D frictionImpulse = tangent * jt_applied;

                        // Apply immediately!
                        bodyA->velocity = bodyA->velocity - (frictionImpulse * invMassA);
                        bodyA->angularVelocity -= cross2D(rA, frictionImpulse) * invInertiaA;
                        bodyB->velocity = bodyB->velocity + (frictionImpulse * invMassB);
                        bodyB->angularVelocity += cross2D(rB, frictionImpulse) * invInertiaB;
                    }
                }
            }
        }
    }

    // =========================================================
    // STAGE IV: POSITIONAL CORRECTION (Run ONCE at the end)
    // =========================================================
    for (auto& col : activeCollisions) {
        RigidBody* bodyA = col.bodyA;
        RigidBody* bodyB = col.bodyB;
        manifold& m = col.m;

        float invMassA = bodyA->getInvMass();
        float invMassB = bodyB->getInvMass();
        float totalInvMass = invMassA + invMassB;

        // === THE FIX: DIVIDE-BY-ZERO SAFETY NET ===
        if (totalInvMass == 0.0f) continue;

        const float percent = 0.8f; // Strong lift
        const float slop = 0.01f;   // Tight allowed overlap 
        
        float penetration = std::max(m.penetration - slop, 0.0f);
        vec2D correction = m.normal * ((penetration * percent) / totalInvMass);

        // Push objects out linearly; the iterations of velocity rotation solved the hovering!
        bodyA->position = bodyA->position - (correction * invMassA);
        bodyB->position = bodyB->position + (correction * invMassB);
    }

    // =========================================================
    // STAGE V: CLEANUP
    // =========================================================
    for(RigidBody* body: bodies) {
        body->clearForces();
    }
}