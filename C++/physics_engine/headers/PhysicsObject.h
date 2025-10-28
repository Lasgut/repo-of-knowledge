#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

#include <windows.h>

class PhysicsObject {
public:
    float posX, posY;     // Position
    float velX, velY;     // Velocity
    float accX, accY;     // Acceleration
    float mass;           // Mass of the object
    float radius;         // Radius for circular collision detection

    PhysicsObject(float x, float y, float r, float m)
        : posX(x), posY(y), velX(0), velY(0), accX(0), accY(0), radius(r), mass(m) {}

    void ApplyForce(float fx, float fy) {
        accX += fx / mass;
        accY += fy / mass;
    }

    void Update(float deltaTime) {
        // Update velocity based on acceleration
        velX += accX * deltaTime;
        velY += accY * deltaTime;

        // Update position based on velocity
        posX += velX * deltaTime;
        posY += velY * deltaTime;

        // Reset acceleration for the next frame
        accX = 0;
        accY = 0;
    }

    void Draw(HDC hdc) {
        // Draw a circle representing the physics object
        Ellipse(hdc, posX - radius, posY - radius, posX + radius, posY + radius);
    }
};

#endif
