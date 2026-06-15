#include <SFML/Graphics.hpp>
#include "World.h"
#include "circle.h"
#include "box.h"

int main() {
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Engine");
    window.setFramerateLimit(60);
    
    World my_world(vec2D(0, 200.f)); 
    
    // Dynamic Objects (Your balls)
    circle ballShape(20);
    RigidBody fay(50, vec2D(300, 300), &ballShape);
    RigidBody khang(50, vec2D(500, 300), &ballShape);
    
    // Give them initial velocity to collide in the center
    fay.velocity = vec2D(150, -50);  // Added a little upward tilt to see them bounce around!
    khang.velocity = vec2D(-150, -50);
    
    my_world.addBody(&fay);
    my_world.addBody(&khang);

    // -------------------------------------------------------------
    // CREATE THE PHYSICAL BOX ARENA 
    // -------------------------------------------------------------
    box horizontalWallShape(800.f, 40.f); 
    box verticalWallShape(40.f, 600.f);   

    RigidBody floor(0.0f, vec2D(400.f, 580.f), &horizontalWallShape); 
    RigidBody ceiling(0.0f, vec2D(400.f, 20.f), &horizontalWallShape);
    RigidBody leftWall(0.0f, vec2D(20.f, 300.f), &verticalWallShape);
    RigidBody rightWall(0.0f, vec2D(780.f, 300.f), &verticalWallShape);

    my_world.addBody(&floor);
    my_world.addBody(&ceiling);
    my_world.addBody(&leftWall);
    my_world.addBody(&rightWall);

    // -------------------------------------------------------------
    // SFML VISUAL RENDERING SETUP
    // -------------------------------------------------------------
    sf::CircleShape circle1(20);
    circle1.setFillColor(sf::Color::Green);
    circle1.setOrigin({20.f, 20.f});
    
    sf::CircleShape circle2(20);
    circle2.setFillColor(sf::Color::Red);
    circle2.setOrigin({20.f, 20.f});

    // Create the visual rectangles for the walls
    sf::RectangleShape sfFloor({800.f, 40.f});
    sfFloor.setOrigin({400.f, 20.f}); // Origin in center
    sfFloor.setPosition({400.f, 580.f});
    sfFloor.setFillColor(sf::Color(100, 100, 100)); // Dark Gray

    sf::RectangleShape sfCeiling({800.f, 40.f});
    sfCeiling.setOrigin({400.f, 20.f});
    sfCeiling.setPosition({400.f, 20.f});
    sfCeiling.setFillColor(sf::Color(100, 100, 100));

    sf::RectangleShape sfLeft({40.f, 600.f});
    sfLeft.setOrigin({20.f, 300.f});
    sfLeft.setPosition({20.f, 300.f});
    sfLeft.setFillColor(sf::Color(100, 100, 100));

    sf::RectangleShape sfRight({40.f, 600.f});
    sfRight.setOrigin({20.f, 300.f});
    sfRight.setPosition({780.f, 300.f});
    sfRight.setFillColor(sf::Color(100, 100, 100));



    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Step the physics math forward
        my_world.step(0.016f);
        

        // Translate the physics math to the SFML painter
        circle1.setPosition({fay.position.x, fay.position.y});
        circle2.setPosition({khang.position.x, khang.position.y});

        window.clear(sf::Color::Black);
        window.draw(circle1);
        window.draw(circle2);
        window.draw(sfFloor);
        window.draw(sfCeiling);
        window.draw(sfLeft);
        window.draw(sfRight);
        window.display();
    }

    return 0;
}