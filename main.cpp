/// @file main.cpp
/// @brief Entry point for testing the physics engine.
#include "vector2D.h"
#include <iostream>

int main() {
    vec2D v1(6, 7);
    vec2D v2 = v1 * 3;
    std::cout << v2.x << ", " << v2.y << "\n";
    return 0;
}