#pragma once

// An enum to quickly check what type of shape we are dealing with
enum class shapeType {
    CIRCLE,
    BOX
};

class shape {
public:
    shapeType type;
    
    // A virtual destructor is required for C++ base classes!
    virtual ~shape() = default; 
};