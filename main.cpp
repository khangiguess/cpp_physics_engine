/// @file main.cpp
/// @brief Entry point for testing the physics engine.
#include "vector2D.h"
#include <iostream>


// Run this to test: g++ main.cpp vector2D.cpp -o main && ./main

int main() {
    vec2D a(6, 7);
    vec2D b(1, 1);

    a -= b;
    std::cout << a.x << ", " << a.y << "\n"; // should be 5, 6
    a += b;
    std::cout << a.x << ", " << a.y << "\n"; // should be 6, 7
}