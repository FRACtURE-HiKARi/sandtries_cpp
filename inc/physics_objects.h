#pragma once
#include "types.h"
#include "nodes.h"
#include "utils.h"

class PhysicsObject: public Node {
protected:
    float mass;
    Vec2 local_centroid;
    float m_inertia;
};

class Collider: public PhysicsObject {
protected:
    // TODO: what the fuck troublesome using STL
    // with abstract class...
    virtual void initInertia();
public:
    explicit Collider(float mass);
    Vec2 getCentroidOffset() const;
    float getMass() const;
    float getInertia(const Vec2 &host_centroid) const;
};

class BallCollider: public Collider {
protected:
    float radius;
    void initInertia() override;
public:
    explicit BallCollider(float mass, float radius);
};

class RectangleCollider: public Collider {
protected:
    float h;
    float w;
    void initInertia() override;
public:
    explicit RectangleCollider(float mass, float width, float height);
};

// TODO: fill this collider
class PolygonCollider: public Collider {

};

class PhysicsBody: public PhysicsObject {
protected:
    Vec2 global_centroid;
    Mat2 rotation;
    std::list <Collider> collider_list;

public:
    Vec2 globalToLocalVec(const Vec2 &p);
    Vec2 localToGlobalVec(const Vec2 &p);
    void addCollider(Collider &collider);
    virtual void addForce(const Vec2 &f) = 0;
    virtual void zeroAll();
};

class StaticBody: public PhysicsBody {

public:
    void addForce(const Vec2& force) override;
};

class RigidBody: public PhysicsBody {
    Vec2 velocity;
    Vec2 acceleration;
    Vec2 force;
    float torque;
    float angular_velocity;
    float angular_acceleration;

    Vec2 integratePosition(float dt);
    Mat2 integrateRotation(float dt);

public:
    // TODO: a useful constructor
    Pose integration(float dt);
    void zeroAll() override;
    void addForce(const Vec2 &force) override;
    void addTorque(float torque);
    void addTorque(Vec2 force, Vec2 offset);
};
