#pragma once
#include "types.h"
#include "nodes.h"
#include "utils.h"

class PhysicsObject: public Node {

};

class CollisionBox: public PhysicsObject {

};

class PhysicsBody: public PhysicsObject {
    CollisionBox collisionBox;
    Vec2 global_centroid;
    Vec2 local_centroid;
    std::set < PhysicsBody > collider_list;
    Vec2 globalToLocalVec(const Vec2 &p);
    Vec2 localToGlobalVec(const Vec2 &p);
};

class StaticBody: public PhysicsBody {

};

class RigidBody: public PhysicsBody {
    float mass;
    Vec2 velocity;
    Vec2 accleration;
    Vec2 force;
    float torque;
    float m_ineratia;
    float rotation;
    float angular_velocity;
    float angular_accleration;
    void updateGlobalCentroid();
};
