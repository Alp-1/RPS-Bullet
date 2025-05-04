#include <btBulletDynamicsCommon.h>  // Bullet Physics core, linter error
#include <GL/glew.h>                 // OpenGL Extension Wrangler
#include <GLFW/glfw3.h>              // GLFW for window/context and input
#include <GL/glu.h>                  // GLU for gluLookAt
#include <vector>                    // std::vector for lists
#include <iostream>                  // std::cout, std::cerr
#include <opencv2/opencv.hpp>        // OpenCV for image processing, linter error
#include <algorithm>                 // for std::minmax_element

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


int heightmapToTerrain(const std::string& path,
                       TerrainDescriptor& descriptor)
{
  cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (image.empty()) return -1;

  int width  = image.cols;
  int height = image.rows;
  descriptor.rows = height;
  descriptor.cols = width;
  descriptor.heightData.resize(height * width);
  descriptor.heightScale = 1.0f;  // TO DO: temporary placeholder needs to be driven by the URDF description of the agent

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float normalized = image.at<uchar>(y,x) / 255.0f;
      descriptor.heightData[y*width + x] = normalized * descriptor.heightScale;
    }
  }

  // compute min/max
  auto mm = std::minmax_element(
               descriptor.heightData.begin(),
               descriptor.heightData.end());
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


// Initialize OpenGL state after context creation
void initGL() {
    glewInit();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
}

int main() {
    // 1. Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }
    GLFWwindow* window = glfwCreateWindow(
        800, 600,
        "Bullet Physics Terrain Demo",
        nullptr, nullptr
    );
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << "\n";

    // 2. Initialize GL and Bullet
    initGL();
    initPhysics();

    // 3. Build only the terrain
    TerrainDescriptor terrain;
    terrain.gravity     = btVector3(0, -9.81f, 0);
    terrain.planeNormal = btVector3(0,  1.0f, 0);
    terrain.planeOffset = 0.0f;
    buildTerrain(dynamicsWorld, terrain);

    // 4. Main render loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        dynamicsWorld->stepSimulation(1.0f / 60.0f);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // set up projection matrix
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(
            45.0f,               // field of view
            800.0f / 600.0f,     // aspect ratio
            0.1f,                // near clip
            1000.0f              // far clip
        );

        // set up modelview matrix (camera)
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(
            1.0f, 15.0f,  0.0f,  // eye position
            0.0f,  0.0f,  0.0f,  // look-at point
            0.0f,  1.0f,  0.0f   // up vector
        );

        // draw reference axes
        glBegin(GL_LINES);
          glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(10,0,0);
          glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,10,0);
          glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,10);
        glEnd();

        // draw ground quad
        glColor3f(0.7f, 0.7f, 0.7f);
        glBegin(GL_QUADS);
          glVertex3f(-5, 0, -5);
          glVertex3f(-5, 0,  5);
          glVertex3f( 5, 0,  5);
          glVertex3f( 5, 0, -5);
        glEnd();

        glfwSwapBuffers(window);
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

    // 6. Terminate GLFW
    glfwTerminate();
    return 0;
}
