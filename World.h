#pragma once
#include <vector>
#include "RigidBody.h"

/**
 * @brief Represents the physics simulation world.
 * 
 * Contains all rigid bodies and steps the simulation
 * forward in time. Apply forces before calling step().
 */
class World {
public:
    vec2D gravity;                   ///< gravitational acceleration applied to all bodies (m/s^2)
    std::vector<RigidBody*> bodies;  ///< all rigid bodies in the simulation

    /**
     * @brief Creates a World with a given gravity vector.
     * @param gravity - gravitational acceleration (e.g. vec2D(0, -9.8))
     */
    World(vec2D gravity);

    /**
     * @brief Adds a rigid body to the simulation.
     * @param body - pointer to the RigidBody to add
     */
    void addBody(RigidBody* body);

    /**
     * @brief Steps the simulation forward by dt seconds.
     * 
     * Applies gravity to all bodies, updates their
     * positions and velocities, then clears forces.
     * 
     * @param dt - time elapsed since last frame (seconds)
     */
    void step(float dt);
};