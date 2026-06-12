#pragma once
#include "shape.h"

class circle : public shape {
public:
    float radius;

    circle (float r) {
        this->radius = r;
        this->type = shapeType::CIRCLE;
    }
};