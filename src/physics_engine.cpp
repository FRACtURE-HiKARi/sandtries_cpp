//
// Created by 13364 on 2024/10/15.
//

#include "physics_engine.h"
#include <graphics.h>

void BasicBroadPhase::addAABB(AABB *aabb) {
    aabbs.push_back(aabb);
}

void BasicBroadPhase::update() {/* do nothing */ }

// n-squared complexity
ColliderPairList& BasicBroadPhase::getAllPairs() {
    colliderPairList.clear();
    for (auto a = aabbs.begin(); a != aabbs.end(); a++) {
        for (auto b = std::next(a); b != aabbs.end(); b++) {
            if (AABB::meets(*a, *b)) {
                colliderPairList.emplace_back(
                        (*a)->collider, (*b)->collider
                );
            }
        }
    }
    return colliderPairList;
}

Collider* BasicBroadPhase::pick(const Vec2& point) {
    for (AABB* aabb: aabbs) {
        if (aabb->contains(point))
            return aabb->collider;
    }
    return nullptr;
}

void BasicBroadPhase::query(Vec2 &point, ColliderList &result) {
    for (AABB* aabb: aabbs) {
        if (aabb->contains(point))
            result.push_back(aabb->collider);
    }
}

RayCastResult BasicBroadPhase::rayCast(const Ray2 &ray) {
    struct Result {
        Collider* collider;
        float t;
        Vec2 normal;
        bool operator< (const Result& rhs) const {
            return t < rhs.t;
        }
    };
    ColliderList candidates;
    for (AABB* aabb: aabbs) {
        if (aabb->testRay(ray))
            candidates.push_back(aabb->collider);
    }
    std::vector<Result> all_hits;
    float t;
    Vec2 normal;
    bool hit = false;
    // hit = start + t * dir;
    for (Collider* candidate: candidates) {
        if (candidate->testRay(ray, &t, &normal)) {
            all_hits.push_back({
                candidate,
                t,
                normal
            });
            hit = true;
        }
    }
    if (!hit) {
        return {
            false,
            nullptr,
            {0},
            {0}
        };
    }
    std::sort(all_hits.begin(), all_hits.end());
    Result first = all_hits.at(0);
    return {
            true,
            first.collider,
            ray.start + ray.direction * first.t,
            first.normal
    };
}

Vec2 CollisionHandler::getMinkowskiDiff(const Collider *c1, const Collider *c2, const Vec2 &dir) {
     return c1->supportVec(dir) - c2->supportVec(dir*-1);
}

// TODO: test the correctness of this algorithm.
bool CollisionHandler::GJK(ColliderPair pair) {
    const Collider* c1 = pair.first;
    const Collider* c2 = pair.second;
    typedef std::pair<Vec2, Vec2> Edge;
    typedef struct Triangle {
        std::list<Vec2> vertices;
        void append(const Vec2 &v) {
            vertices.push_front(v);
        }
        bool handle(Vec2 *d) {
            if (vertices.size() == 2)
                return lineCase(d);
            return triangleCase(d);
        }
        bool lineCase(Vec2 *d) {
            auto it = vertices.begin();
            Vec2 a = *(it++);
            Vec2 b = *it;
            *d = normTo(a, b, {0});
            return false;
        }
        bool triangleCase(Vec2 *d) {
            auto it = vertices.begin();
            Vec2 a = *(it++);
            Vec2 b = *(it++);
            Vec2 c = *it;
            Vec2 n_ab = normTo(a, b, c);
            Vec2 n_ac = normTo(a, c, b);
            if (n_ab * a > 0) {
                vertices.remove(c);
                *d = n_ab * -1;
                return false;
            }
            else if (n_ac * a > 0) {
                vertices.remove(c);
                *d = n_ac * -1;
                return false;
            }
            return true;
        }
    } Triangle;
    Triangle triangle;
    Vec2 dir = Calculations::unit(c1->getPosition() - c2->getPosition());
    Vec2 v = getMinkowskiDiff(c1, c2, dir);
    triangle.append(v);
    dir = Calculations::unit(v * -1);
    while (true) {
        v = getMinkowskiDiff(c1, c2, dir);
        if (v * dir < 0) return false;
        triangle.append(v);
        if (triangle.handle(&dir))
            return true;
    }
}

Vec2 CollisionHandler::normTo(const Vec2 &A, const Vec2 &B, const Vec2 &O) {
    if (A == B) return {0};
    return Calculations::normal(A-O, A - B) * -1;
}

void PhysicsEngine::applyGravity() {
    Vec2 gravityForce = {0, gravity};
    for (RigidBody* object: rigid_bodies) {
        object->addForce(gravityForce);
    }
}

PhysicsEngine::PhysicsEngine() : PhysicsEngine(0) {}

PhysicsEngine::PhysicsEngine(float gravity) {
    this->gravity = gravity;
}

void PhysicsEngine::updateObjects(float dt) {
    if (gravity != 0) applyGravity();
    for (RigidBody* rb: rigid_bodies) {
        rb->integration(dt);
        rb->clear();
    }
    ColliderPairList candidates = broadPhase.getAllPairs();
    for (ColliderPair pair: candidates) {
        circle(20, 20, 10);
        if (CollisionHandler::GJK(pair))
            fillcircle(20, 20, 10);
    }

}

void PhysicsEngine::addRigid(RigidBody &rigidBody) {
    rigid_bodies.push_back(&rigidBody);
    AABBList list;
    rigidBody.pushAllAABBs(list);
    for (AABB *aabb: list)
        broadPhase.addAABB(aabb);
}

void PhysicsEngine::addStatic(StaticBody &staticBody) {
    static_bodies.push_back(&staticBody);
    AABBList list;
    staticBody.pushAllAABBs(list);
    for (AABB *aabb: list)
        broadPhase.addAABB(aabb);
}
