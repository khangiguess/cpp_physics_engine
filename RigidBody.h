#pragma once
#include <math.h>
#include "vector2D.h"
#include "shape.h"

/**
 * @brief Represents a physical object in 2D space with linear and angular capabilities.
 */
class RigidBody {
public:
    // Linear properties
    vec2D position;     ///< position in 2D space (meters)
    vec2D velocity;     ///< speed and direction (meters/second)
    vec2D acceleration; ///< current acceleration (meters/second^2)
    float mass;         ///< mass of the object (kg)
    float invMass;      ///< 1/mass (or 0 for static bodies)

    // Angular properties
    float angle;           ///< orientation in radians
    float angularVelocity; ///< spin speed (rad/sec)
    float torque;          ///< rotational force accumulator
    float inertia;         ///< moment of inertia (rotational mass)
    float invInertia;      ///< 1/inertia (or 0 for static bodies)

    // Material properties
    shape* m_shape;        ///< shape of the rigid body for collision detection
    float restitution;     ///< Coefficient of elasticity
    float staticFriction;  ///< static friction coefficient
    float dynamicFriction; ///< dynamic friction coefficient
    
    /**
     * @brief Creates a RigidBody with optional rotational defaults.
     */
    RigidBody(float mass, vec2D startPos, shape* s, float bounciness = 0.5f, 
              float staticFriction = 0.5f, float dynamicFriction = 0.3f, 
              float startAngle = 0.0f);

    /**
     * @brief Steps the linear and angular simulation forward by dt seconds.
     */
    void update(float dt);

    /**
     * @brief Applies a force acting directly on the center of mass.
     */
    void applyForce(vec2D force);

    /**
     * @brief Applies a torque (rotational force) directly.
     */
    void applyTorque(float t);

    /**
     * @brief Resets linear and angular force accumulators.
     */
    void clearForces();

    float getMass() const { return mass; }
    float getInvMass() const { return invMass; }
    float getInertia() const { return inertia; }
    float getInvInertia() const { return invInertia; }
};