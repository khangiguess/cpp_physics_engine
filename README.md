# 2D Rigid Body Physics Engine

## A simple 2D rigid body physics engine written entirely in C++ capable of simulating interation between objects.

This project is created for my educational purpose, and greatly inspired by the Youtube series by Two-Bit Coding for a physics engine in C. Therefore, there will be similar concepts and logic that I borrowed in order to complete this project.

## Features
* A core physics engine
* Contact points detection (commented out)
* Vectors visualization for velocities and forces (commented out in the code)
* An interactive renderer for for testing with object spawning


--------------------------------------------------------------------------------------



## Architecture and limitations
    Due to limited resources, this project can only cover a certain amount of crucial factors of a phyics engine

* **Collision Detection:** Separating Axis Theorem (SAT) is used for overlap detection for oriented bounding boxes and circles.
* **Sutherland-Hodgman Clipping:** To generate accurate multi-point contact manifolds and independent penetration depths.
* **Sequential Impulses:** Solves complex multi-body interactions iteratively. This approach may hinder a more accurate and precise simulation of system with multiple objects and/or sensitive to changes. 
*Note: The engine uses a "cold-starting" approach (intra-frame accumulators rather than cross-frame caching). Because of this, simulating massive multi-object systems may require higher iteration counts to maintain precision and stack stability.*
* **Friction & Damping:** Calculates dynamic and static friction as a 2D cross-product, accurately accounting for torque and normal force constraints. It also includes basic velocity damping to simulate air resistance and prevent continuous floating-point drift.
* **Custom parameters:** These includes overlapping and reactionary force percentage that can be adjusted to the like of the users. They will help the user to tweak the engine in case the logic itself has some minor flaws.



## Dependencies
* C++17 or higher
* [SFML](https://www.sfml-dev.org/) (Simple and Fast Multimedia Library)

## Author
**Khang Minh Trinh** Computer Engineering, Ho Chi Minh City University of Technology