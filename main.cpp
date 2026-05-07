#include <SFML/Graphics.hpp>
#include "World.h"

int main() {
    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics Engine");
    window.setFramerateLimit(60);
    
    World my_world(vec2D(0, 981.f));
    RigidBody fay(50, vec2D(400, 300));
    my_world.addBody(&fay);

    sf::CircleShape circle(20);
    circle.setFillColor(sf::Color::Green);
    circle.setOrigin({20.f, 20.f});  // set origin to center of circle

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        my_world.step(0.016f);
        if (fay.position.y > 600 - 20) {  // 600 = window height, 20 = circle radius
            fay.position.y = 600 - 20;
            fay.velocity.y *= -0.8f;  // bounce and lose energy
        }

        circle.setPosition({fay.position.x, fay.position.y});

        window.clear(sf::Color::Black);
        window.draw(circle);
        window.display();
    }

    return 0;
}