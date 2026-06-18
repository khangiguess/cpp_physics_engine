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
const vec2D GRAVITY(0.0f, 200.f);          // Gravity vector (x, y)

// Arena Wall Configurations
const float WALL_THICKNESS        = 20.f;  // Width/thickness of boundary walls
const sf::Color WALL_COLOR        = sf::Color(100, 100, 100); // Dark Gray

// Dynamic Object A (Fay) Config
const float OBJ_A_MASS            = 100.f;
const float OBJ_A_BOUNCINESS      = 0.5f;
const float OBJ_A_STATIC_FRIC     = 0.2f;
const float OBJ_A_DYNAMIC_FRIC    = 0.1f;
const vec2D OBJ_A_START_POS       = vec2D(300.f, 580.f); //Origin is set at top left corner
const vec2D OBJ_A_START_VEL       = vec2D(300.f, 0.f);
const sf::Color OBJ_A_COLOR       = sf::Color::Green;

// Dynamic Object B (Khang) Config
const float OBJ_B_MASS            = 100.f;
const float OBJ_B_BOUNCINESS      = 0.5f;
const float OBJ_B_STATIC_FRIC     = 0.2f;
const float OBJ_B_DYNAMIC_FRIC    = 0.1f;
const vec2D OBJ_B_START_POS       = vec2D(500.f, 580.f); //Origin is set at top left corner
const vec2D OBJ_B_START_VEL       = vec2D(-300.f, 0.f);
const sf::Color OBJ_B_COLOR       = sf::Color::Red;











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
                  float bounce = 0.4f, float staticF = 0.4f, float dynamicF = 0.15f) {
    
    // Create physical RigidBody and add to physics engine
    RigidBody* body = new RigidBody(mass, pos, physShape, bounce, staticF, dynamicF);
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
    
    // Store all actors (dynamic objects & walls) in a single vector
    std::vector<Actor> actors;

    // -------------------------------------------------------------
    // Change shapes here!
    // -------------------------------------------------------------
    // shape* shapeA = new circle(20.f);
    // shape* shapeB = new circle(20.f);
    shape* shapeA = new box(35.f, 15.f); 
    shape* shapeB = new box(35.f, 15.f);

    // Create the physical + visual dynamic actors
    actors.push_back(createActor(my_world, OBJ_A_MASS, OBJ_A_START_POS, OBJ_A_START_VEL, shapeA, OBJ_A_COLOR, OBJ_A_BOUNCINESS, OBJ_A_STATIC_FRIC, OBJ_A_DYNAMIC_FRIC));
    actors.push_back(createActor(my_world, OBJ_B_MASS, OBJ_B_START_POS, OBJ_B_START_VEL, shapeB, OBJ_B_COLOR, OBJ_B_BOUNCINESS, OBJ_B_STATIC_FRIC, OBJ_B_DYNAMIC_FRIC));

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

        // 1. Advance the mathematical physics step
        my_world.step(TIME_STEP);
        
        // 2. Synchronize visual positions with the updated physics positions
        for (auto& actor : actors) {
            actor.visualShape->setPosition({actor.body->position.x, actor.body->position.y});
        }

        // 3. Clear window, render all actors dynamically, and display frame
        window.clear(sf::Color::Black);
        
        for (const auto& actor : actors) {
            window.draw(*(actor.visualShape));
        }
        
        window.display();
    }

    // -------------------------------------------------------------
    // CLEANUP MEMORY (Good C++ resource management)
    // -------------------------------------------------------------
    for (auto& actor : actors) {
        delete actor.body;
        delete actor.physicalShape;
        delete actor.visualShape;
    }

    return 0;
}