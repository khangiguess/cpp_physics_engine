#pragma once
#include <math.h>

/**
 * @brief Represents a 2D vector with basic math operations.
 *
 * Stores x and y components as floats. Supports addition,
 * subtraction, scalar multiplication, magnitude, normalization,
 * and dot product.
 */
class vec2D {
public:
    float x; ///< x component
    float y; ///< y component

    /**
     * @brief Default constructor. Initializes x and y to 0.
     */
    vec2D();

    /**
     * @brief Creates a vec2D with the given x and y components.
     * @param x - x component
     * @param y - y component
     */
    vec2D(float x, float y);

    /**
     * @brief Adds two vectors component-wise.
     * @param a - the vector to add
     * @return a new vec2D equal to this + a
     */
    vec2D operator+(const vec2D& a) const;

    /**
     * @brief Adds another vector to this one in place
     * @param a - the vector to add
     * @return reference to this vec2D after addition
     */
    vec2D& operator+=(const vec2D& a);

    /**
     * @brief Subtracts two vectors component-wise.
     * @param a - the vector to subtract
     * @return a new vec2D equal to this - a
     */
    vec2D operator-(const vec2D& a) const;

    /**
     * @brief Subtracts another vector to this one in place
     * @param a - the vector to subtract
     * @return reference to this vec2D after subtraction
     */
    vec2D& operator-=(const vec2D& a);

    /**
     * @brief Scales the vector by a scalar.
     * @param scalar - the value to multiply by
     * @return a new vec2D equal to this * scalar
     */
    vec2D operator*(float scalar) const;

    /**
     * @brief Computes the length of the vector.
     * @return sqrt(x^2 + y^2)
     */
    float magnitude() const;

    /**
     * @brief Returns a unit vector pointing in the same direction.
     *
     * Returns (0, 0) if the vector has zero length.
     *
     * @return normalized vec2D with magnitude 1
     */
    vec2D normalize() const;

    /**
     * @brief Computes the dot product with another vector.
     * @param a - the other vector
     * @return x*a.x + y*a.y
     */
    float dot(const vec2D& a) const;
};