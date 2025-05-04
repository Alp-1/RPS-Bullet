// src/plane_render.cpp
/*
#include <btBulletDynamicsCommon.h>
#include <GLFW/glfw3.h>
#include <GL/gl.h>

// Simple btIDebugDraw implementation
struct DebugDrawer : public btIDebugDraw {
    int m = DBG_DrawWireframe;
    void drawLine(const btVector3& f, const btVector3& t, const btVector3& c) override {
        glColor3f(c.x(), c.y(), c.z());
        glBegin(GL_LINES);
          glVertex3f(f.x(), f.y(), f.z());
          glVertex3f(t.x(), t.y(), t.z());
        glEnd();
    }
    void drawContactPoint(const btVector3&, const btVector3&, btScalar, int, const btVector3&) override {}
    void reportErrorWarning(const char* msg) override { fprintf(stderr, "Warn: %s\n", msg); }
    void draw3dText(const btVector3&, const char*) override {}
    void setDebugMode(int dm) override { m = dm; }
    int  getDebugMode() const override { return m; }
};

int main() {
    // Initialize GLFW
    if (!glfwInit()) return 1;
    GLFWwindow* win = glfwCreateWindow(800, 600, "Plane Debug Draw", nullptr, nullptr);
    if (!win) return 1;
    glfwMakeContextCurrent(win);
    glEnable(GL_DEPTH_TEST);

    // Bullet world
    btDefaultCollisionConfiguration cfg;
    btCollisionDispatcher dispatcher(&cfg);
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &cfg);
    world.setGravity(btVector3(0, -9.81f, 0));

    // Attach debug drawer
    DebugDrawer debugDrawer;
    world.setDebugDrawer(&debugDrawer);
    debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe);

    // Add ground plane
    btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0,1,0), 0);
    btDefaultMotionState* ms = new btDefaultMotionState(btTransform::getIdentity());
    btRigidBody::btRigidBodyConstructionInfo info(0.0f, ms, plane);
    btRigidBody* body = new btRigidBody(info);
    world.addRigidBody(body);

    // Main loop
    while (!glfwWindowShouldClose(win)) {
        world.stepSimulation(1/60.0f);

        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Orthographic top‐down camera
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = w / float(h);
        glOrtho(-10*aspect, 10*aspect, -10, 10, -10, 10);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef(90, 1, 0, 0);

        world.debugDrawWorld();

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return 0;
}
*/