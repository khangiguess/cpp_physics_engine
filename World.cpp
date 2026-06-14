#include "World.h"
#include "collision.h"

World::World(vec2D gravity) {
    this->gravity = gravity;
}

void World::addBody(RigidBody* body) {
    bodies.push_back(body);
}

void World::step(float dt) {
    for(RigidBody* body: bodies) {
        body->applyForce(vec2D(gravity.x * body->getMass(), gravity.y * body->getMass()));
        body->update(dt);

        for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];

            
            manifold m = generateManifold(*(bodyA->m_shape), *(bodyB->m_shape), bodyA->position, bodyB->position);

            if (m.isColliding) {
                float invMassA = 1.0f / bodyA->getMass();
                float invMassB = 1.0f / bodyB->getMass();
                float totalInvMass = invMassA + invMassB;

                if (totalInvMass > 0.0f) {
                    float moveRatioA = invMassA / totalInvMass;
                    float moveRatioB = invMassB / totalInvMass;

                    bodyA->position = bodyA->position - (m.normal * m.penetration * moveRatioA);
                    bodyB->position = bodyB->position + (m.normal * m.penetration * moveRatioB);
                }
                
            }
        }
    }

        body->clearForces();
    }
}