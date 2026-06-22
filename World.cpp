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
    debugLines.clear();
    // STAGE I: INTEGRATION
    for(RigidBody* body: bodies) {
        body->applyForce(vec2D(gravity.x * body->getMass(), gravity.y * body->getMass()));
        //Directional vectors for testing
        //drawLine(body->position, body->position + (body->acceleration * 0.2f), sf::Color(255, 165, 0));
        body->update(dt);
        
        ///Directional vectors for testing
        //drawLine(body->position, body->position + (body->velocity * 0.3f), sf::Color::Green);
    }

    // STAGE II:MANIFOLD GENERATION
    std::vector<CollisionData> activeCollisions;
    
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];

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

    //Directional vectors for testing
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
    // STAGE III: THE ITERATIVE SOLVER
    const int ITERATIONS = 30; // Number of passes to stabilize the constraints

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

                //NORMAL IMPULSE (Bounce & Penetration)
                vec2D vA_contact = bodyA->velocity + vec2D(-bodyA->angularVelocity * rA.y, bodyA->angularVelocity * rA.x);
                vec2D vB_contact = bodyB->velocity + vec2D(-bodyB->angularVelocity * rB.y, bodyB->angularVelocity * rB.x);
                vec2D relativeVelocity = vB_contact - vA_contact;
                
                float velocityAlongNormal = relativeVelocity.dot(m.normal);

                float rA_cross_N = cross2D(rA, m.normal);
                float rB_cross_N = cross2D(rB, m.normal);

                float invMassSum = invMassA + invMassB + 
                    (rA_cross_N * rA_cross_N * invInertiaA) + 
                    (rB_cross_N * rB_cross_N * invInertiaB);

                float restitution = std::min(bodyA->restitution, bodyB->restitution);
                if (std::abs(velocityAlongNormal) < 25.0f) restitution = 0.0f; 

                // Calculate the raw iteration impulse
                float j = -(1.0f + restitution) * velocityAlongNormal / invMassSum;

                float oldNormalImpulse = m.normalImpulse[k];
                // Clamp the total accumulated impulse to >= 0 (Objects can push apart, never pull together)
                m.normalImpulse[k] = std::max(oldNormalImpulse + j, 0.0f);
                
                // Only apply the difference between the clamped total and the old total
                float delta_j = m.normalImpulse[k] - oldNormalImpulse;
                vec2D normalPush = m.normal * delta_j;

                //Normal Delta
                bodyA->velocity = bodyA->velocity - (normalPush * invMassA);
                bodyA->angularVelocity -= cross2D(rA, normalPush) * invInertiaA;
                bodyB->velocity = bodyB->velocity + (normalPush * invMassB);
                bodyB->angularVelocity += cross2D(rB, normalPush) * invInertiaB;

                // TANGENTIAL IMPULSE (Friction)
                // Recalculate velocity after normal impulse alters it
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

                    //raw friction impulse
                    float jt = -velocityAlongTangent / invMassSumTangent;
                    
                    float mu = std::sqrt(bodyA->dynamicFriction * bodyA->dynamicFriction + bodyB->dynamicFriction * bodyB->dynamicFriction);
                    float baselineGravityImpulse = 3.2f / (invMassA + invMassB);
                    float maxFriction = (m.normalImpulse[k] + baselineGravityImpulse) * mu;

                    float oldTangentImpulse = m.tangentImpulse[k];
                    // Clamp friction
                    m.tangentImpulse[k] = std::clamp(oldTangentImpulse + jt, -maxFriction, maxFriction);
                    
                    float delta_jt = m.tangentImpulse[k] - oldTangentImpulse;
                    vec2D frictionPush = tangent * delta_jt;

                    // Apply Tangential Delta
                    bodyA->velocity = bodyA->velocity - (frictionPush * invMassA);
                    bodyA->angularVelocity -= cross2D(rA, frictionPush) * invInertiaA;
                    bodyB->velocity = bodyB->velocity + (frictionPush * invMassB);
                    bodyB->angularVelocity += cross2D(rB, frictionPush) * invInertiaB;
                }
            }
        }
    }

    // STAGE IV: POSITIONAL CORRECTION (Run ONCE at the end)
    for (auto& col : activeCollisions) {
        RigidBody* bodyA = col.bodyA;
        RigidBody* bodyB = col.bodyB;
        manifold& m = col.m;

        float invMassA = bodyA->getInvMass();
        float invMassB = bodyB->getInvMass();
        float totalInvMass = invMassA + invMassB;

        if (totalInvMass == 0.0f) continue;

        const float percent = 0.5f; // lift
        const float slop = 0.2f;   // allowed overlap 
        
        float penetration = std::max(m.penetration - slop, 0.0f);
        vec2D correction = m.normal * ((penetration * percent) / totalInvMass);

        // Push objects out linearly; the iterations of velocity rotation solved the hovering!
        bodyA->position = bodyA->position - (correction * invMassA);
        bodyB->position = bodyB->position + (correction * invMassB);
    }

    // STAGE V: CLEANUP
    for(RigidBody* body: bodies) {
        body->clearForces();
    }
}