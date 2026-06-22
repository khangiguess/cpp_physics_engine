#pragma once
#include <vector>
#include "RigidBody.h"
#include <SFML/Graphics.hpp>

/**
 * @brief Represents the physics simulation world.
 * 
 * Contains all rigid bodies and steps the simulation
 * forward in time. Apply forces before calling step().
 */
// Directional vectors for testing
struct DebugLine {
    vec2D start;
    vec2D end;
    sf::Color color;
};

class World {
public:
    vec2D gravity;                   ///< gravitational acceleration applied to all bodies (m/s^2)
    std::vector<RigidBody*> bodies;  ///< all rigid bodies in the simulation
    // Directional vectors for testing
    std::vector<DebugLine> debugLines;
    void drawLine(vec2D start, vec2D end, sf::Color color = sf::Color::White) {
        debugLines.push_back({start, end, color});
    }

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