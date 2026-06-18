#include "World.h"
#include "collision.h"
#include <algorithm>

World::World(vec2D gravity) {
    this->gravity = gravity;
}

void World::addBody(RigidBody* body) {
    bodies.push_back(body);
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
    // STEP 2: COLLISION DETECTION & RESOLUTION
    // ---------------------------------------------------------
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];
            
            manifold m = generateManifold(*(bodyA->m_shape), *(bodyB->m_shape), bodyA->position, bodyB->position);

            if (m.isColliding) {
                float invMassA = bodyA->getInvMass();
                float invMassB = bodyB->getInvMass();
                float totalInvMass = invMassA + invMassB;

                if (totalInvMass > 0.0f) {
                    // --- POSITIONAL CORRECTION (Separation) ---
                    float moveRatioA = invMassA / totalInvMass;
                    float moveRatioB = invMassB / totalInvMass;

                    bodyA->position = bodyA->position - (m.normal * m.penetration * moveRatioA);
                    bodyB->position = bodyB->position + (m.normal * m.penetration * moveRatioB);

                    // Calculate relative velocity at collision point
                    vec2D relativeVelocity = bodyB->velocity - bodyA->velocity;
                    float velocityAlongNormal = relativeVelocity.dot(m.normal);

                    // --- IMPULSE RESPONSE (Bounce & Friction) ---
                    // Only resolve if objects are actively moving towards each other
                    if (velocityAlongNormal < 0.0f) {
                        
                        // 1. NORMAL IMPULSE (The Bounce)
                        float restitution = std::min(bodyA->restitution, bodyB->restitution);
                        float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal / totalInvMass;
                        
                        vec2D bounceImpulse = m.normal * impulseMagnitude;
                        bodyA->velocity = bodyA->velocity - (bounceImpulse * invMassA);
                        bodyB->velocity = bodyB->velocity + (bounceImpulse * invMassB);

                        // 2. TANGENTIAL IMPULSE (The Friction)
                        // Note how we use the updated relativeVelocity for friction!
                        vec2D tangent = relativeVelocity - (m.normal * velocityAlongNormal);

                        if (tangent.magnitude() > 0.0001f) {
                            tangent = tangent.normalize();
                            float velocityAlongTangent = relativeVelocity.dot(tangent);

                            float mu_s = std::sqrt(bodyA->staticFriction * bodyA->staticFriction + bodyB->staticFriction * bodyB->staticFriction);
                            float mu_d = std::sqrt(bodyA->dynamicFriction * bodyA->dynamicFriction + bodyB->dynamicFriction * bodyB->dynamicFriction);

                            float jt = -velocityAlongTangent / totalInvMass;

                            vec2D frictionImpulse;

                            // Coulomb's Law
                            if (std::abs(jt) < impulseMagnitude * mu_s) {
                                frictionImpulse = tangent * jt; // Grip
                            } else {
                                frictionImpulse = tangent * -impulseMagnitude * mu_d; // Slide
                            }

                            // Apply friction
                            bodyA->velocity = bodyA->velocity - (frictionImpulse * invMassA);
                            bodyB->velocity = bodyB->velocity + (frictionImpulse * invMassB);
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