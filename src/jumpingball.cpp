#include <btBulletDynamicsCommon.h>
#include <iostream>
#include "envbuilder.cpp"

int main() {
    // 1) Bullet setup
    btDefaultCollisionConfiguration collisionConfig;
    btCollisionDispatcher dispatcher(&collisionConfig);
    btDbvtBroadphase broadphase;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &collisionConfig);

    world.setGravity({0, -9.81f, 0});

    // 2) Ground plane
/*    btCollisionShape* groundShape = new btStaticPlaneShape({0,1,0}, 0);
    btDefaultMotionState* groundMotion = new btDefaultMotionState(btTransform::getIdentity());
    btRigidBody::btRigidBodyConstructionInfo
        groundInfo(0.0, groundMotion, groundShape);
    btRigidBody* groundBody = new btRigidBody(groundInfo);
    world.addRigidBody(groundBody); */

    buildEnv();

    // 3) Dynamic sphere
    btCollisionShape* sphereShape = new btSphereShape(1.0f);
    btDefaultMotionState* sphereMotion =
      new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), {0,10,0}));
    btScalar mass = 1.0f;
    btVector3 inertia(0,0,0);
    sphereShape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo sphereInfo(mass, sphereMotion, sphereShape, inertia);
    btRigidBody* sphereBody = new btRigidBody(sphereInfo);
    world.addRigidBody(sphereBody);

    // 4) Step simulation and print Y‐height
    const int steps = 300;
    for (int i = 0; i < steps; ++i) {
        world.stepSimulation(1.0f/60.0f);
        btTransform trans;
        sphereBody->getMotionState()->getWorldTransform(trans);
        std::cout << "t=" << i*(1.0f/60.0f)
                  << "  height=" << trans.getOrigin().getY() << "\n";
    }

    // 5) Cleanup (in real code you'd delete pointers)
    delete sphereBody;
    delete sphereMotion;
    delete sphereShape;
    delete groundBody;
    delete groundMotion;
    delete groundShape;

    return 0;
}
