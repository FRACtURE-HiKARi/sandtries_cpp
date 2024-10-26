//
// Created by 13364 on 2024/10/15.
//

#pragma once
#include "physics_objects.h"
#include "utils.h"

typedef std::pair<Collider*, Collider*> ColliderPair;
typedef std::list<ColliderPair> ColliderPairList;

typedef struct RayCastResult {
    bool hit;
    Collider* collider;
    Vec2 position;
    Vec2 normal;
}RayCastResult;

class BroadPhase {
public:
    virtual void addAABB(AABB *aabb) = 0;
    virtual void update() = 0;
    virtual ColliderPairList& getAllPairs() = 0;
    virtual Collider* pick(const Vec2& point) = 0;
    virtual void query(Vec2& point, ColliderList& result) = 0;
    virtual RayCastResult rayCast(const Ray2& ray) = 0;
};

// Implementation with very n-squared complexity
class BasicBroadPhase: public BroadPhase {
    std::list<AABB*> aabbs;
    ColliderPairList colliderPairList;
public:
    void addAABB(AABB *aabb) override;
    void update() override;
    ColliderPairList& getAllPairs() override;
    Collider* pick(const Vec2& point) override;
    void query(Vec2& point, ColliderList& result) override;
    RayCastResult rayCast(const Ray2& ray) override;

};

class Renderer;
typedef std::pair<float, Vec3> EPAResult;
typedef std::list<Vec3> Simplex;
typedef std::pair<bool, Simplex> GJKResult;
class CollisionHandler {
public:
    Renderer *renderer = nullptr;
    static Vec3 getMinkowskiDiff(const Collider *c1, const Collider *c2, const Vec3 &dir);
    GJKResult GJK(ColliderPair pair);
    EPAResult EPA(ColliderPair pair, Simplex& s);
    static EPAResult circles(BallCollider* b1, BallCollider* b2);
    static void assignImpulse(ColliderPair pair, EPAResult epaResult);
};

// TODO: collision and resting
class PhysicsEngine {
    BasicBroadPhase broadPhase;
    std::vector <StaticBody*> static_bodies;
    std::vector <RigidBody*> rigid_bodies;
    float gravity;
    void applyGravity();
    Renderer *renderer = nullptr;
    CollisionHandler handler;
public:
    PhysicsEngine();
    explicit PhysicsEngine(float gravity);
    void updateObjects(float dt);
    void addRigid(RigidBody& rigidBody);
    void addStatic(StaticBody& staticBody);
    void setRenderer(Renderer &r);
};
