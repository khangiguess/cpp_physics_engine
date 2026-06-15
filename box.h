#pragma once
#include "shape.h"

class box : public shape {
public:
    float width;
    float height;

    box(float w, float h) {
        this->width = w;
        this->height = h;
        this->type = shapeType::BOX; 
    }
};