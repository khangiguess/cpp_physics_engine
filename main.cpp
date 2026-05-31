#include <SFML/Graphics.hpp>
#include <deque>
#include <cmath>
#include "Simulation.h"

const int W = 800;
const int H = 650;
const int SIM_H = 450;
const float SCALE = 170.0f;
const int TRAIL_MAX = 600;
const float EPSILON = 1e-6f;
const int RENORM_FRAMES = 90;
const int LAMBDA_HISTORY = 500;

sf::Vector2f toScreen(vec2D pos) {
    return {W / 2.f + pos.x * SCALE, SIM_H / 2.f - pos.y * SCALE};
}

int main() {
    sf::RenderWindow window(sf::VideoMode({800, 650}), "N-Body Gravitational Simulation");
    window.setFramerateLimit(60);

    // Figure-8 choreography (Chenciner & Montgomery, 2000)
    RigidBody b1(1.0f, vec2D(-0.97000436f,  0.24308753f));
    b1.velocity = vec2D( 0.46620368f,  0.43236573f);
    RigidBody b2(1.0f, vec2D( 0.97000436f, -0.24308753f));
    b2.velocity = vec2D( 0.46620368f,  0.43236573f);
    RigidBody b3(1.0f, vec2D(0.0f, 0.0f));
    b3.velocity = vec2D(-0.93240737f, -0.86473146f);

    Simulation sim;
    sim.addBody(&b1);
    sim.addBody(&b2);
    sim.addBody(&b3);

    // Shadow simulation — perturbed copy to track divergence
    RigidBody sb1 = b1, sb2 = b2, sb3 = b3;
    sb1.position.x += EPSILON;

    Simulation shadow_sim;
    shadow_sim.addBody(&sb1);
    shadow_sim.addBody(&sb2);
    shadow_sim.addBody(&sb3);

    RigidBody* bodies[3] = {&b1, &b2, &b3};
    RigidBody* shadow[3] = {&sb1, &sb2, &sb3};

    std::deque<sf::Vector2f> trails[3];
    sf::Color colors[3] = {
        sf::Color(255, 100,  80),
        sf::Color( 80, 210, 100),
        sf::Color( 80, 160, 255)
    };

    float lyapunov_sum = 0.f;
    float total_time   = 0.f;
    float lambda       = 0.f;
    int   frame        = 0;
    std::deque<float> lambda_history;

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        sim.step(0.016f);
        shadow_sim.step(0.016f);
        total_time += 0.016f;
        frame++;

        // Full phase-space separation (position + velocity)
        float d2 = 0.f;
        for (int i = 0; i < 3; i++) {
            vec2D dp = shadow[i]->position - bodies[i]->position;
            vec2D dv = shadow[i]->velocity - bodies[i]->velocity;
            d2 += dp.dot(dp) + dv.dot(dv);
        }
        float d = sqrtf(d2);

        if (frame % RENORM_FRAMES == 0 && d > 0.f) {
            lyapunov_sum += logf(d / EPSILON);
            lambda = lyapunov_sum / total_time;

            float scale = EPSILON / d;
            for (int i = 0; i < 3; i++) {
                vec2D dp = shadow[i]->position - bodies[i]->position;
                vec2D dv = shadow[i]->velocity - bodies[i]->velocity;
                shadow[i]->position = bodies[i]->position + dp * scale;
                shadow[i]->velocity = bodies[i]->velocity + dv * scale;
            }
        }

        lambda_history.push_back(lambda);
        if ((int)lambda_history.size() > LAMBDA_HISTORY)
            lambda_history.pop_front();

        for (int i = 0; i < 3; i++) {
            trails[i].push_back(toScreen(bodies[i]->position));
            if ((int)trails[i].size() > TRAIL_MAX)
                trails[i].pop_front();
        }

        window.clear(sf::Color(10, 10, 20));

        // --- Orbit ---
        for (int i = 0; i < 3; i++) {
            if (trails[i].size() < 2) continue;
            sf::VertexArray va(sf::PrimitiveType::LineStrip, trails[i].size());
            for (size_t j = 0; j < trails[i].size(); j++) {
                float alpha = (float)j / trails[i].size() * 220.f;
                va[j] = sf::Vertex{trails[i][j], sf::Color(colors[i].r, colors[i].g, colors[i].b, (uint8_t)alpha)};
            }
            window.draw(va);
        }
        for (int i = 0; i < 3; i++) {
            sf::CircleShape dot(6.f);
            dot.setFillColor(colors[i]);
            dot.setOrigin({6.f, 6.f});
            dot.setPosition(toScreen(bodies[i]->position));
            window.draw(dot);
        }

        // --- Divider ---
        sf::RectangleShape divider({(float)W, 1.f});
        divider.setPosition({0.f, (float)SIM_H});
        divider.setFillColor(sf::Color(50, 50, 70));
        window.draw(divider);

        // --- Lyapunov graph ---
        const float GT = SIM_H + 10.f;
        const float GB = H - 10.f;
        const float GH = GB - GT;
        const float GL = 10.f;
        const float GR = W - 10.f;
        const float GW = GR - GL;

        sf::RectangleShape graph_bg({GW, GH});
        graph_bg.setPosition({GL, GT});
        graph_bg.setFillColor(sf::Color(15, 15, 30));
        graph_bg.setOutlineColor(sf::Color(40, 40, 60));
        graph_bg.setOutlineThickness(1.f);
        window.draw(graph_bg);

        const float LAM_MIN = -0.5f;
        const float LAM_MAX =  3.0f;

        auto lamToY = [&](float lam) -> float {
            float t = (lam - LAM_MIN) / (LAM_MAX - LAM_MIN);
            return GB - t * GH;
        };

        // Zero baseline
        float zero_y = lamToY(0.f);
        sf::Vertex zero_line[2] = {
            sf::Vertex{sf::Vector2f(GL, zero_y), sf::Color(60, 60, 90)},
            sf::Vertex{sf::Vector2f(GR, zero_y), sf::Color(60, 60, 90)}
        };
        window.draw(zero_line, 2, sf::PrimitiveType::Lines);

        // Lambda curve
        if (lambda_history.size() >= 2) {
            sf::VertexArray lva(sf::PrimitiveType::LineStrip, lambda_history.size());
            for (size_t j = 0; j < lambda_history.size(); j++) {
                float x = GL + (float)j / (LAMBDA_HISTORY - 1) * GW;
                float y = lamToY(lambda_history[j]);
                y = std::max(GT, std::min(GB, y));
                float alpha = 80.f + (float)j / lambda_history.size() * 175.f;
                lva[j] = sf::Vertex{sf::Vector2f(x, y), sf::Color(255, 200, 50, (uint8_t)alpha)};
            }
            window.draw(lva);

            float lx = GL + (float)(lambda_history.size() - 1) / (LAMBDA_HISTORY - 1) * GW;
            float ly = lamToY(lambda_history.back());
            ly = std::max(GT, std::min(GB, ly));
            sf::CircleShape ldot(4.f);
            ldot.setFillColor(sf::Color(255, 220, 80));
            ldot.setOrigin({4.f, 4.f});
            ldot.setPosition({lx, ly});
            window.draw(ldot);
        }

        window.display();
    }

    return 0;
}
