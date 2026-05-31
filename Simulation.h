#pragma once
#include <vector>
#include "RigidBody.h"

class Simulation {
public:
    float G;
    float softening;
    int substeps;
    std::vector<RigidBody*> bodies;

    Simulation(float G = 1.0f, float softening = 0.05f, int substeps = 100);
    void addBody(RigidBody* body);
    void step(float dt);
};
