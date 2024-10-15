//
// Created by 13364 on 2024/10/15.
//

#include "physics_engine.h"

void PhysicsEngine::applyGravity() {
    Vec2 gravityForce = {0, -gravity};
    for (RigidBody& object: rigid_bodies) {
        object.addForce(gravityForce);
    }
}

PhysicsEngine::PhysicsEngine(float gravity) {
    this->gravity = gravity;
}

void PhysicsEngine::updateObjects(float dt) {
    if (gravity != 0) applyGravity();
}

void PhysicsEngine::addRigid(const RigidBody &rigidBody) {
    rigid_bodies.push_back(rigidBody);
}

void PhysicsEngine::addStatic(const StaticBody &staticBody) {
    static_bodies.push_back(staticBody);
}
