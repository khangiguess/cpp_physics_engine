#include <SFML/Graphics.hpp>
#include <vector>
#include "World.h"
#include "circle.h"
#include "box.h"

// =============================================================================
// 1. CONFIGURATION PARAMETERS
// =============================================================================

// Window Configurations
const unsigned int WINDOW_WIDTH   = 800;
const unsigned int WINDOW_HEIGHT  = 600;
const float TIME_STEP             = 0.016f; // Standard physics time step (60 FPS)

// Global Physics Setting
const vec2D GRAVITY(0.0f, 200.f);         

// Arena Wall Configurations
const float WALL_THICKNESS        = 20.f;  // Width/thickness of boundary walls
const sf::Color WALL_COLOR        = sf::Color(100, 100, 100); // Dark Gray












// =============================================================================
// 2. RENDERING PIPELINE STRUCTURES & HELPERS
// =============================================================================

/**
 * @brief Binds a physical RigidBody and its visual SFML Shape representation together.
 * This simplifies updating and rendering in loops.
 */
struct Actor {
    RigidBody* body;         ///< Pointer to physical rigid body
    shape* physicalShape;    ///< Pointer to collision shape (stored for cleanup)
    sf::Shape* visualShape;  ///< Pointer to SFML drawable representation
};

/**
 * @brief Helper factory to instantiate a physical body and automatically generate 
 * its matching SFML visual counterpart.
 */
Actor createActor(World& world, float mass, vec2D pos, vec2D vel, shape* physShape, sf::Color color, 
                  float bounce = 0.4f, float staticF = 0.4f, float dynamicF = 0.15f, float startAngle = 0.0f) {
    
    // Create physical RigidBody and add to physics engine
    RigidBody* body = new RigidBody(mass, pos, physShape, bounce, staticF, dynamicF, startAngle);
    body->velocity = vel;
    world.addBody(body);

    sf::Shape* visualShape = nullptr;

    // Detect collision shape type to automatically construct matching SFML shape
    if (physShape->type == shapeType::CIRCLE) {
        float r = static_cast<circle*>(physShape)->radius;
        sf::CircleShape* c = new sf::CircleShape(r);
        c->setOrigin({r, r}); // Set origin to center for proper rotation/position mapping
        c->setFillColor(color);
        visualShape = c;
    } 
    else if (physShape->type == shapeType::BOX) {
        float w = static_cast<box*>(physShape)->width;
        float h = static_cast<box*>(physShape)->height;
        sf::RectangleShape* r = new sf::RectangleShape({w, h});
        r->setOrigin({w / 2.0f, h / 2.0f}); // Set origin to center for proper alignment
        r->setFillColor(color);
        visualShape = r;
    }

    return { body, physShape, visualShape };
}












// =============================================================================
// 3. MAIN GAME ENTRY POINT
// =============================================================================

int main() {
    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Physics Engine");
    window.setFramerateLimit(60);
    
    // Create physical world with configured gravity
    World my_world(GRAVITY); 
    std::vector<Actor> actors;

    const int TOWER_HEIGHT = 8;     // Number of boxes in the stack
    const float BOX_SIZE = 40.f;    // 40x40 pixel boxes

    for (int i = 0; i < TOWER_HEIGHT; i++) {
        // Create a new shape for every actor to prevent double-free memory crashes during cleanup
        shape* boxShape = new box(BOX_SIZE, BOX_SIZE);
        
        // Stack directly in the center, dropping  with 5-pixel gap between each
        vec2D startPos(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT - 100.f - (i * (BOX_SIZE + 5.f)));
        
        // Alternate colors
        sf::Color color = (i % 2 == 0) ? sf::Color::Red : sf::Color::Blue;

        actors.push_back(createActor(
            my_world, 
            100.f,           // Mass
            startPos,        // Position
            vec2D(0.f, 0.f), // Start Velocity
            boxShape,        // Physical Shape
            color,           // Color
            0.0f,            // ZERO bounciness so they try to rest immediately
            0.5f,            // High Static Friction
            0.3f,            // Standard Dynamic Friction
            0.0f             // Angle
        ));
    }

    // -------------------------------------------------------------
    // DYNAMIC ARENA BOUNDARY WALL GENERATION (Self-correcting size)
    // -------------------------------------------------------------
    // Instantiate boundary shapes according to Window dimensions
    box* topWallShape    = new box((float)WINDOW_WIDTH, WALL_THICKNESS);
    box* bottomWallShape = new box((float)WINDOW_WIDTH, WALL_THICKNESS);
    box* leftWallShape   = new box(WALL_THICKNESS, (float)WINDOW_HEIGHT);
    box* rightWallShape  = new box(WALL_THICKNESS, (float)WINDOW_HEIGHT);

    // Add 4 static walls (mass = 0.0f) using automatic position math
    actors.push_back(createActor(my_world, 0.0f, vec2D(WINDOW_WIDTH / 2.0f, WALL_THICKNESS / 2.0f), vec2D(0, 0), topWallShape, WALL_COLOR));
    actors.push_back(createActor(my_world, 0.0f, vec2D(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT - WALL_THICKNESS / 2.0f), vec2D(0, 0), bottomWallShape, WALL_COLOR));
    actors.push_back(createActor(my_world, 0.0f, vec2D(WALL_THICKNESS / 2.0f, WINDOW_HEIGHT / 2.0f), vec2D(0, 0), leftWallShape, WALL_COLOR));
    actors.push_back(createActor(my_world, 0.0f, vec2D(WINDOW_WIDTH - WALL_THICKNESS / 2.0f, WINDOW_HEIGHT / 2.0f), vec2D(0, 0), rightWallShape, WALL_COLOR));

    // -------------------------------------------------------------
    // SIMULATION & RENDERING LOOP
    // -------------------------------------------------------------
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        //Advance the mathematical physics step
        my_world.step(TIME_STEP);
        
        //Synchronize visual positions and rotations with the updated physics positions
        for (auto& actor : actors) {
            actor.visualShape->setPosition({actor.body->position.x, actor.body->position.y});
            
            // Convert radians to degrees for SFML rotation mapping
            float angleDegrees = actor.body->angle * 180.0f / 3.14159265f;
            actor.visualShape->setRotation(angleDegrees);
        }

        //Clear window, render all actors dynamically, and display frame
        window.clear(sf::Color::Black);
        
        for (const auto& actor : actors) {
            window.draw(*(actor.visualShape));
        }
        // Directional vectors for testing
        /*
        for (const auto& line : my_world.debugLines) {
            sf::Vertex sfmlLine[] = {
                sf::Vertex(sf::Vector2f(line.start.x, line.start.y), line.color),
                sf::Vertex(sf::Vector2f(line.end.x, line.end.y), line.color)
            };
            window.draw(sfmlLine, 2, sf::Lines);
        }*/   
        window.display();
    }

    // CLEANUP MEMORY
    for (auto& actor : actors) {
        delete actor.body;
        delete actor.physicalShape;
        delete actor.visualShape;
    }

    return 0;
}