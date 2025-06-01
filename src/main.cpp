#include <SFML/Graphics.hpp>
#include <btBulletDynamicsCommon.h>
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include <vector>                    // std::vector for lists
#include <iostream>                  // std::cout, std::cerr
#include <algorithm>                 // for std::minmax_element
#include <random>
#include "lidarSensor.h"


// Terrain descriptor holds gravity and plane parameters
struct TerrainDescriptor {
    btVector3 gravity;      // gravity vector
    btVector3 planeNormal;  // ground plane normal
    btScalar  planeOffset;  // ground plane offset

    std::string heightmapPath; // path to heightmap image

    std::vector<float>    heightData;   // ← needs to be added
    int                   rows;         // ← needs to be added
    int                   cols;         // ← needs to be added
    float                 heightScale;  // ← needs to be added
    float                 minHeight;    // optional
    float                 maxHeight;    // optional
};

// Bullet globals
btDefaultCollisionConfiguration*      collisionConfiguration;
btCollisionDispatcher*                dispatcher;
btBroadphaseInterface*                overlappingPairCache;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld*              dynamicsWorld;
std::vector<btCollisionShape*>        collisionShapes;

// Initialize Bullet world shell (no bodies yet)
void initPhysics() {
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher            = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache  = new btDbvtBroadphase();
    solver                = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(
        dispatcher,
        overlappingPairCache,
        solver,
        collisionConfiguration
    );
}


int heightmapToTerrain(const std::string& path, TerrainDescriptor& descriptor)
{
    sf::Image image;
    if (!image.loadFromFile(path)) {
        return -1;
    }

    sf::Vector2u size = image.getSize();
    int width = size.x;
    int height = size.y;

    descriptor.cols = width;
    descriptor.rows = height;
    descriptor.heightData.resize(height * width);
    descriptor.heightScale = 1.0f;  // TO DO: Replace with agent's URDF info

    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            sf::Color pixel = image.getPixel({x, y});
            // Convert to grayscale assuming the image is already grayscale
            float normalized = pixel.r / 255.0f;
            descriptor.heightData[y * width + x] = normalized * descriptor.heightScale;
        }
    }

    // Compute min/max
    auto mm = std::minmax_element(
        descriptor.heightData.begin(), descriptor.heightData.end());

    descriptor.minHeight = *mm.first;
    descriptor.maxHeight = *mm.second;

    return 0;
}

// Build terrain only: set gravity and add infinite plane
// NOTE: descriptor must be non-const so you can fill its heightData
void buildTerrain(
    btDiscreteDynamicsWorld* world,
    TerrainDescriptor& descriptor   // <<–– drop the const
) {
    // 1) set gravity
    world->setGravity(descriptor.gravity);

    // 2) load the heights into descriptor; result==0 means success
    int result = heightmapToTerrain(descriptor.heightmapPath, descriptor);

    // 3) if loader succeeded AND we have data, build heightfield
    if (result == 0 && !descriptor.heightData.empty()) {
        // ––– build btHeightfieldTerrainShape here –––
        // // Create Bullet heightfield shape
        auto heightfieldShape = new btHeightfieldTerrainShape(
            descriptor.cols, descriptor.rows,
            descriptor.heightData.data(),
            descriptor.heightScale,                // height scale
            descriptor.minHeight, descriptor.maxHeight,        // min/max height
            1,                   // up axis (Y = 1)
            PHY_FLOAT,           // data type
            false                // flip quad edges
        );
    }
    else {
        // ––– fall back to the infinite plane –––
        btCollisionShape* groundShape =
            new btStaticPlaneShape(descriptor.planeNormal,
                                   descriptor.planeOffset);
        collisionShapes.push_back(groundShape);

        btTransform t; t.setIdentity();
        auto* motion = new btDefaultMotionState(t);
        btRigidBody::btRigidBodyConstructionInfo info(
            0.0, motion, groundShape, btVector3(0,0,0)
        );
        world->addRigidBody(new btRigidBody(info));
    }
}

int main() {
    initPhysics();

    // 3. Build only the terrain
    TerrainDescriptor terrain;
    terrain.gravity     = btVector3(0, -9.81f, 0);
    terrain.planeNormal = btVector3(0,  1.0f, 0);
    terrain.planeOffset = 0.0f;
    terrain.heightmapPath = "heightmap.png";
    buildTerrain(dynamicsWorld, terrain);

    // LIDAR INSTANCE
    if (terrain.cols == 0 || terrain.rows == 0) {
        std::cerr << "Error: terrain dimensions are invalid." << std::endl;
        return -1;
    }
    // LIDAR sensor at a fixed point above the terrain (e.g. 1 meter high)
    btVector3 lidarPos(terrain.cols / 2.0f, 1.0f, terrain.rows / 2.0f); // Y is up
    // Create Lidar Instance
    lidarSensor lidar(dynamicsWorld, lidarPos, 20, -1.57f, 1.57f, 30.0f); // 180° horizontal scan, 20 beams
    // LIDAR INSTANCE

    const unsigned width = 505;
    const unsigned height = 505;
    auto window = sf::RenderWindow(sf::VideoMode({width, height}), "RPS-Bullet");
    window.setFramerateLimit(144);
    

    sf::Image image("heightmap.png");

    sf::Texture texture;
    texture.loadFromImage(image, false, sf::IntRect({0, 0}, {width, height}));
    sf::Sprite sprite(texture);

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }

        }
//Lidar tester starts
        static float t = 0.0f;
        t += 0.05f; // controls speed

        // Move in a circle on the XZ plane
        float radius = 50.0f;
        float x = terrain.cols / 2.0f + radius * std::cos(t);
        float z = terrain.rows / 2.0f + radius * std::sin(t);
        float y = 1.0f; // fixed height above terrain

        btVector3 newOrigin(x, y, z);
        lidar.setOrigin(newOrigin);

        // Perform a LIDAR scan (every frame for now)
        auto distances = lidar.scan();
        std::cout << "LIDAR scan: ";
        for (float d : distances) {
            std::cout << d << " ";
        }
        std::cout << std::endl;
// Lidar tester ends

        window.clear();
        window.draw(sprite);
        window.display();
    }

    // 5. Cleanup Bullet objects
    for (auto* shape : collisionShapes) {
        delete shape;
    }
    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
    
    return 0;
}
