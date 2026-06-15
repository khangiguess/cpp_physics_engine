/// @file RigidBody.cpp
/// @brief Implementation of the RigidBody class.
#include "RigidBody.h"

RigidBody::RigidBody(float mass, vec2D startPos, shape* s, float bounciness) {
    this->mass = mass;
    this->position = startPos;
    this->velocity = vec2D(0, 0);
    this->acceleration = vec2D(0, 0);
    
    this->m_shape = s;
    this->restitution = bounciness;

    //calculate Inverse Mass
    if (this->mass <= 0.0f) {
        this->invMass = 0.0f; // It's a static wall! It cannot move.
    } else {
        this->invMass = 1.0f / this->mass;
    }
}

void RigidBody::update(float dt) {
    this->velocity += (this->acceleration) * dt;
    this->position += (this->velocity) * dt;
}

void RigidBody::applyForce(vec2D force) {
    if (this->mass > 0.0f) {
        this->acceleration += force * (1.0f / (this->mass));
    }
    // Static bodies (mass == 0) don't accelerate
}

void RigidBody::clearForces() {
    this->acceleration = vec2D();
}
