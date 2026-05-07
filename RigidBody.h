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
    shape* shape;
    
    /**
     * @brief Creates a RigidBody at a given position with a given mass.
     * @param mass - mass of the object (kg)
     * @param startPos - starting position as a vec2D
     */
    RigidBody(float mass, vec2D startPos);

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
};