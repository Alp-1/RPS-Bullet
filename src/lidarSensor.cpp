// LidarSensor.cpp

#include "lidarSensor.h"
#include <cmath>         // For std::cos and std::sin
#include <vector>        // For std::vector (though included via LidarSensor.h, explicit can be good)

// Constructor Implementation
lidarSensor::lidarSensor(btCollisionWorld* world,
                         const btVector3& origin,
                         int numBeams,
                         float minAngle,
                         float maxAngle,
                         float maxRange)
    : m_world(world),       // Initialize member variable m_world with the 'world' parameter
      m_origin(origin),     // Initialize m_origin with the 'origin' parameter
      m_numBeams(numBeams),   // Initialize m_numBeams
      m_minAngle(minAngle),   // Initialize m_minAngle
      m_maxAngle(maxAngle),   // Initialize m_maxAngle
      m_maxRange(maxRange)    // Initialize m_maxRange
{
    // The constructor body.
    // This is where you could put additional setup code that needs to run
    // when a LidarSensor object is created, after the members are initialized.
    // For example, if you had a member vector for caching results and wanted to reserve space:
    // if (m_numBeams > 0) {
    //     m_someCacheVector.reserve(m_numBeams);
    // }
}

// setOrigin Method Implementation
void lidarSensor::setOrigin(const btVector3& newOrigin) {
    // Update the sensor's internal origin position
    m_origin = newOrigin;
}

// scan Method Implementation
std::vector<float> lidarSensor::scan() {
    std::vector<float> distances; // Vector to store the measured distances

    // If there are no beams to cast, return an empty vector
    if (m_numBeams <= 0) {
        return distances;
    }

    // Pre-allocate memory for the distances to improve performance
    distances.reserve(m_numBeams);

    float angleStep = 0.0f;
    // Calculate the angular step between beams
    // Avoid division by zero if m_numBeams is 1
    if (m_numBeams > 1) {
        // If multiple beams, spread them across the angle range
        angleStep = (m_maxAngle - m_minAngle) / static_cast<float>(m_numBeams - 1);
    }
    // If m_numBeams == 1, angleStep remains 0.0f, and the loop will use m_minAngle for the single beam.

    // Iterate for each beam
    for (int i = 0; i < m_numBeams; ++i) {
        float currentAngle;
        if (m_numBeams == 1) {
            // If only one beam, place it at the start of the angle range,
            // or you could choose the middle: (m_minAngle + m_maxAngle) / 2.0f
            currentAngle = m_minAngle;
        } else {
            currentAngle = m_minAngle + i * angleStep;
        }

        // Calculate the direction vector for the current beam (2D scan in XY plane)
        // Assumes Z is the "up" axis if this is a horizontal scan.
        // If Y is "up", you might use dir(std::cos(currentAngle), 0.0f, std::sin(currentAngle))
        btVector3 rayDir(std::cos(currentAngle), std::sin(currentAngle), 0.0f);
        rayDir.normalize(); // Ensure it's a unit vector, though cos/sin on a unit circle should already be.

        // Define the start and end points for the raycast
        btVector3 rayFrom = m_origin;
        btVector3 rayTo = m_origin + rayDir * m_maxRange;

        // Create a callback object to receive raycast results
        // This callback will find the closest object hit by the ray
        btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);

        // Perform the raycast in the Bullet physics world
        m_world->rayTest(rayFrom, rayTo, rayCallback);

        // Process the result of the raycast
        if (rayCallback.hasHit()) {
            // If the ray hit an object, calculate the distance to the hit point
            // m_closestHitFraction is a value between 0 (hit at 'rayFrom') and 1 (hit at 'rayTo')
            float hitDistance = m_maxRange * rayCallback.m_closestHitFraction;
            distances.push_back(hitDistance);
        } else {
            // If the ray did not hit any object within m_maxRange, record the maximum range
            distances.push_back(m_maxRange);
        }
    }

    return distances; // Return the vector of measured distances
}