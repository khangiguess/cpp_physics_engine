#include <SFML/Graphics.hpp>
#include "World.h"
#include "circle.h"

int main() {
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Engine");
    window.setFramerateLimit(60);
    
    World my_world(vec2D(0, 200.f));  // reduced gravity for better collision demo
    
    // Create rigid bodies with circle shapes
    circle shape1(20);
    circle shape2(20);
    
    RigidBody fay(50, vec2D(300, 100), &shape1);
    RigidBody khang(50, vec2D(500, 100), &shape2);
    
    my_world.addBody(&fay);
    my_world.addBody(&khang);

    // 3. Create visual circles for rendering
    sf::CircleShape circle1(20);
    circle1.setFillColor(sf::Color::Green);
    circle1.setOrigin({20.f, 20.f});
    
    sf::CircleShape circle2(20);
    circle2.setFillColor(sf::Color::Red);
    circle2.setOrigin({20.f, 20.f});

    // Give them initial velocity to collide in the center
    fay.velocity = vec2D(100, 0);
    khang.velocity = vec2D(-100, 0);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Step the physics math forward
        my_world.step(0.016f);
        
        // Hardcoded floor bounds (We will replace this with Static Bodies later!)
        if (fay.position.y > 600 - 20) {
            fay.position.y = 600 - 20;
            fay.velocity.y *= -0.8f;
        }
        if (khang.position.y > 600 - 20) {
            khang.position.y = 600 - 20;
            khang.velocity.y *= -0.8f;
        }

        // Translate the physics math to the SFML painter
        circle1.setPosition({fay.position.x, fay.position.y});
        circle2.setPosition({khang.position.x, khang.position.y});

        window.clear(sf::Color::Black);
        window.draw(circle1);
        window.draw(circle2);
        window.display();
    }

    return 0;
}