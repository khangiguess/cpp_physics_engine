#include "RigidBody.h"
#include "circle.h"
#include "box.h"

RigidBody::RigidBody(float mass, vec2D startPos, shape* s, float bounciness, 
                     float staticFriction, float dynamicFriction, float startAngle) {
    this->mass = mass;
    this->position = startPos;
    this->velocity = vec2D(0.0f, 0.0f);
    this->acceleration = vec2D(0.0f, 0.0f);
    
    this->angle = startAngle;
    this->angularVelocity = 0.0f;
    this->torque = 0.0f;

    this->m_shape = s;
    this->restitution = bounciness;
    this->staticFriction = staticFriction;
    this->dynamicFriction = dynamicFriction;

    if (this->mass <= 0.0f) {
        this->invMass = 0.0f;
        this->inertia = 0.0f;
        this->invInertia = 0.0f;
    } else {
        this->invMass = 1.0f / this->mass;

        //Moment of Inertia (Rotational Inertia)
        if (s->type == shapeType::CIRCLE) {
            float r = static_cast<circle*>(s)->radius;
            // Solid Cylinder/Disk formula: I = 0.5 * m * r^2
            this->inertia = 0.5f * this->mass * r * r;
        } 
        else if (s->type == shapeType::BOX) {
            float w = static_cast<box*>(s)->width;
            float h = static_cast<box*>(s)->height;
            // Solid Cuboid formula: I = (1/12) * m * (w^2 + h^2)
            this->inertia = (1.0f / 12.0f) * this->mass * (w * w + h * h);
        } else {
            this->inertia = 1.0f; // Default fallback
        }

        this->invInertia = 1.0f / this->inertia;
    }
}

void RigidBody::update(float dt) {
    this->velocity += (this->acceleration) * dt;
    this->position += (this->velocity) * dt;

    float angularAcceleration = this->torque * this->invInertia;
    this->angularVelocity += angularAcceleration * dt;
    this->angle += this->angularVelocity * dt;
    
    //Clamp micro-velocities (Linear ONLY)
    if (this->velocity.magnitude() < 0.1f) this->velocity = vec2D(0, 0);
    //Kill micro-rotation
    if (std::abs(this->angularVelocity) < 0.1f) this->angularVelocity = 0.0f;
}

void RigidBody::applyForce(vec2D force) {
    if (this->mass > 0.0f) {
        this->acceleration += force * this->invMass;
    }
}

void RigidBody::applyTorque(float t) {
    if (this->mass > 0.0f) {
        this->torque += t;
    }
}

void RigidBody::clearForces() {
    this->acceleration = vec2D();
    this->torque = 0.0f;
}