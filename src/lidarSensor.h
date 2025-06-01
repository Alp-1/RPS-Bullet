//
// Created by Aix on 6/1/25.
//

#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <vector>                 // For std::vector (used in scan method return type)
#include <btBulletDynamicsCommon.h> // For Bullet Physics types like btCollisionWorld and btVector3

class lidarSensor {
public:
    // Constructor:
    // Takes a pointer to Bullet’s collision world, the sensor's initial position,
    // the number of laser beams, the scan's angular range (min/max angle in radians),
    // and the maximum distance each ray can measure.
    lidarSensor(
        btCollisionWorld* world,      // 1) A pointer to Bullet’s collision world, used to cast rays
        const btVector3& origin,      // 2) The sensor’s position in world coordinates
        int numBeams,                 // 3) How many laser beams (rays) you’ll fire each scan
        float minAngle,               // 4) The starting angle of the scan (in radians)
        float maxAngle,               // 5) The ending angle of the scan (in radians)
        float maxRange                // 6) The maximum distance (in meters) each ray can measure
    );

    // Perform one full scan and return a vector of distances in meters.
    // Each element in the vector corresponds to a beam's measured distance.
    std::vector<float> scan();

    // Move the sensor to a new position in the world.
    void setOrigin(const btVector3& newOrigin);

private:
    // Member variables to store the sensor's configuration and state:
    btCollisionWorld* m_world;    // Pointer to the Bullet physics world for ray casting
    btVector3         m_origin;   // Current position of the sensor in world coordinates
    int               m_numBeams; // Number of beams per scan
    float             m_minAngle; // Minimum angle of the scan sweep (radians)
    float             m_maxAngle; // Maximum angle of the scan sweep (radians)
    float             m_maxRange; // Maximum range of the laser beams (meters)

    // (Optional: If you were planning to have a pre-reserved vector for scan results
    // as discussed for optimization, you might declare it here, e.g.:
    // std::vector<float> m_scanResultsCache;
    // But the implementation we discussed created the vector locally in scan())
};

#endif // LIDAR_SENSOR_H
