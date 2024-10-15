//
// Created by 13364 on 2024/10/15.
//

#pragma once
#include "physics_objects.h"
#include "utils.h"

class BroadPhase {

};

// TODO: collision and st
class PhysicsEngine {
    std::list <StaticBody> static_bodies;
    std::list <RigidBody> rigid_bodies;
    float gravity;
    void applyGravity();
public:
    PhysicsEngine(float gravity);
    void updateObjects(float dt);
    void addRigid(const RigidBody& rigidBody);
    void addStatic(const StaticBody& staticBody);
};
