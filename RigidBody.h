#pragma once 
#include <math.h>
#include <vector2D.h>

class RigidBody{
    public:
    vec2D position;
    vec2D velocity;
    vec2D acceleration;
    float mass;

    RigidBody(float mass, vec2D startPos);
    void update(float dt);
    void applyForce(vec2D force);
};