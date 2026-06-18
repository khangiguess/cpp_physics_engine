#include "World.h"
#include "collision.h"
#include <algorithm>

World::World(vec2D gravity) {
    this->gravity = gravity;
}

void World::addBody(RigidBody* body) {
    bodies.push_back(body);
}

// 2D Cross Product Helper: Scalar output representing the rotational moment of two 2D vectors
inline float cross2D(const vec2D& a, const vec2D& b) {
    return a.x * b.y - a.y * b.x;
}

void World::step(float dt) {
    // ---------------------------------------------------------
    // STEP 1: INTEGRATION (Move everything first)
    // ---------------------------------------------------------
    for(RigidBody* body: bodies) {
        body->applyForce(vec2D(gravity.x * body->getMass(), gravity.y * body->getMass()));
        body->update(dt);
    }

    // ---------------------------------------------------------
    // STEP 2: COLLISION DETECTION & RESOLUTION (With Angular Mechanics)
    // ---------------------------------------------------------
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];
            
            // LINKED TO SAT: Passing the angles into the manifold generator!
            manifold m = generateManifold(*(bodyA->m_shape), *(bodyB->m_shape), bodyA->position, bodyB->position, bodyA->angle, bodyB->angle);

            if (m.isColliding) {
                float invMassA = bodyA->getInvMass();
                float invMassB = bodyB->getInvMass();
                float invInertiaA = bodyA->getInvInertia();
                float invInertiaB = bodyB->getInvInertia();
                float totalInvMass = invMassA + invMassB;

                if (totalInvMass > 0.0f) {
                    // --- POSITIONAL CORRECTION (Separation) ---
                    float moveRatioA = invMassA / totalInvMass;
                    float moveRatioB = invMassB / totalInvMass;

                    bodyA->position = bodyA->position - (m.normal * m.penetration * moveRatioA);
                    bodyB->position = bodyB->position + (m.normal * m.penetration * moveRatioB);

                    // --- ANGULAR VELOCITY RESOLUTION ---
                    // Vectors from center of mass to the contact point (lever arms)
                    vec2D rA = m.contactPoint - bodyA->position;
                    vec2D rB = m.contactPoint - bodyB->position;

                    // Calculate point velocities: V_point = V_linear + (omega x r)
                    // In 2D, (omega x r) is equivalent to a perpendicular vector scaled by omega: (-omega*r.y, omega*r.x)
                    vec2D vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
                    vec2D vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);

                    // Relative velocity at the exact contact point
                    vec2D relativeVelocity = vB_contact - vA_contact;
                    float velocityAlongNormal = relativeVelocity.dot(m.normal);

                    // Only resolve if objects are actively moving towards each other
                    if (velocityAlongNormal < 0.0f) {
                        
                        // Rotational resistance along the normal axis
                        float rA_cross_N = cross2D(rA, m.normal);
                        float rB_cross_N = cross2D(rB, m.normal);

                        // Total inverse mass including rotational resistance (inertia terms)
                        float totalRotationalInvMass = invMassA + invMassB + 
                            (rA_cross_N * rA_cross_N * invInertiaA) + 
                            (rB_cross_N * rB_cross_N * invInertiaB);

                        // 1. NORMAL IMPULSE (The Bounce)
                        float restitution = std::min(bodyA->restitution, bodyB->restitution);
                        float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / totalRotationalInvMass;
                        
                        vec2D bounceImpulse = m.normal * impulseMagnitude;

                        // Apply normal impulse to linear AND angular velocities
                        bodyA->velocity = bodyA->velocity - (bounceImpulse * invMassA);
                        bodyA->angularVelocity -= rA_cross_N * impulseMagnitude * invInertiaA;

                        bodyB->velocity = bodyB->velocity + (bounceImpulse * invMassB);
                        bodyB->angularVelocity += rB_cross_N * impulseMagnitude * invInertiaB;

                        // 2. TANGENTIAL IMPULSE (The Friction)
                        // Recalculate relative velocity after normal impulse has been applied
                        vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
                        vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);
                        relativeVelocity = vB_contact - vA_contact;

                        vec2D tangent = relativeVelocity - (m.normal * relativeVelocity.dot(m.normal));

                        if (tangent.magnitude() > 0.0001f) {
                            tangent = tangent.normalize();
                            float velocityAlongTangent = relativeVelocity.dot(tangent);

                            float rA_cross_T = cross2D(rA, tangent);
                            float rB_cross_T = cross2D(rB, tangent);

                            float totalRotationalInvMassTangent = invMassA + invMassB + 
                                (rA_cross_T * rA_cross_T * invInertiaA) + 
                                (rB_cross_T * rB_cross_T * invInertiaB);

                            float jt = -velocityAlongTangent / totalRotationalInvMassTangent;

                            float mu_s = std::sqrt(bodyA->staticFriction * bodyA->staticFriction + bodyB->staticFriction * bodyB->staticFriction);
                            float mu_d = std::sqrt(bodyA->dynamicFriction * bodyA->dynamicFriction + bodyB->dynamicFriction * bodyB->dynamicFriction);

                            vec2D frictionImpulse;
                            float jt_applied = 0.0f;

                            // Coulomb's Law
                            if (std::abs(jt) < impulseMagnitude * mu_s) {
                                jt_applied = jt;
                            } else {
                                // FIXED BUG: Removed rogue negative sign that caused infinite velocity gain!
                                jt_applied = impulseMagnitude * mu_d * (jt > 0.0f ? 1.0f : -1.0f);
                            }

                            frictionImpulse = tangent * jt_applied;

                            // Apply friction to linear AND angular states
                            bodyA->velocity = bodyA->velocity - (frictionImpulse * invMassA);
                            bodyA->angularVelocity -= rA_cross_T * jt_applied * invInertiaA;

                            bodyB->velocity = bodyB->velocity + (frictionImpulse * invMassB);
                            bodyB->angularVelocity += rB_cross_T * jt_applied * invInertiaB;
                        }
                    }
                }
            }
        }
    }

    // ---------------------------------------------------------
    // STEP 3: CLEANUP (Clear forces for the next frame)
    // ---------------------------------------------------------
    for(RigidBody* body: bodies) {
        body->clearForces();
    }
}