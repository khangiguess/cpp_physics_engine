#pragma once
#include <math.h>
#include "vector2D.h"
#include "shape.h"

/**
 * @brief Represents a physical object in 2D space.
 *
 * Stores position, velocity, acceleration and mass.
 * Call update() each frame to simulate movement and
 * applyForce() to push the object around.
 */
class RigidBody {
public:
    vec2D position;     ///< position in 2D space (meters)
    vec2D velocity;     ///< speed and direction (meters/second)
    vec2D acceleration; ///< current acceleration (meters/second^2)
    float mass;         ///< mass of the object (kg)
    float invMass;
    shape* m_shape;     ///< shape of the rigid body for collision detection
    float restitution; // Coefficient of elasticity
    float staticFriction;  // 0.5f as default
    float dynamicFriction; // 0.3f as default
    
    /**
     * @brief Creates a RigidBody.
     * @param mass - mass of the object (0.0f makes it an immovable static object)
     * @param startPos - starting position
     * @param s - pointer to the shape geometry
     * @param bounciness - how bouncy the object is (default 0.5)
     */
    RigidBody(float mass, vec2D startPos, shape* s, float bounciness = 0.5f, float staticFriction = 0.5f, float dynamicFriction = 0.3f);

    /**
     * @brief Steps the simulation forward by dt seconds.
     *
     * Updates velocity using acceleration, then position
     * using velocity. Call once per frame.
     *
     * @param dt - time elapsed since last frame (seconds)
     */
    void update(float dt);

    /**
     * @brief Applies a force to the object using F = ma.
     *
     * Accumulates into acceleration. Call clearForces()
     * at the end of each frame to reset.
     *
     * @param force - force vector to apply (Newtons)
     */
    void applyForce(vec2D force);

    /**
     * @brief Resets acceleration to zero.
     *
     * Call at the end of each frame after update()
     * so forces don't accumulate across frames.
     */
    void clearForces();

    float getMass() const { return mass; }
    float getInvMass() const { return invMass; }
};