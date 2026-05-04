/// @file RigidBody.cpp
/// @brief Implementation of the RigidBody class.
#include "RigidBody.h"

RigidBody::RigidBody(float mass, vec2D startPos) {
    this->mass = mass;
    this->position = startPos;
    this->acceleration = vec2D();
    this->velocity = vec2D();
}

void RigidBody::update(float dt) {
    this->velocity += (this->acceleration) * dt;
    this->position += (this->velocity) * dt;
}

void RigidBody::applyForce(vec2D force) {
    this->acceleration += force * (1.0f / (this->mass));
}

void RigidBody::clearForces() {
    this->acceleration = vec2D();
}
