//
// Created by 13364 on 2024/10/15.
//

#include "physics_engine.h"
#include "game_engine.h"
#include <graphics.h>
#include <sstream>

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

Vec3 CollisionHandler::getMinkowskiDiff(const Collider *c1, const Collider *c2, const Vec3 &dir) {
     return c1->supportVec(dir) - c2->supportVec(dir*-1);
}

// TODO: test the correctness of this algorithm.
GJKResult CollisionHandler::GJK(ColliderPair pair) {
    const Collider* c1 = pair.first;
    const Collider* c2 = pair.second;
    typedef struct Triangle {
        Simplex vertices;
        bool handle(Vec3 *d) {
            if (vertices.size() == 2)
                return lineCase(d);
            return triangleCase(d);
        }
        bool lineCase(Vec3 *d) {
            auto it = vertices.begin();
            Vec3 a = *(it++);
            Vec3 b = *it;
            *d = normTo(a, b, {0});
            return false;
        }
        bool triangleCase(Vec3 *d) {
            auto it = vertices.begin();
            Vec3 a = *(it++);
            Vec3 b = *(it++);
            Vec3 c = *it;
            Vec3 n_ab = normTo(a, b, c);
            Vec3 n_ac = normTo(a, c, b);
            if (n_ab * a > 0) {
                vertices.remove(c);
                *d = n_ab * -1;
                return false;
            }
            else if (n_ac * a > 0) {
                vertices.remove(b);
                *d = n_ac * -1;
                return false;
            }
            return true;
        }
    } Triangle;
    Triangle triangle;
    Vec3 dir = Calculations::unit(c1->global_centroid() - c2->global_centroid());
    Vec3 V = getMinkowskiDiff(c1, c2, dir);
    triangle.vertices.push_front(V);
    dir = Calculations::unit(V * -1);
    while (true) {
        V = getMinkowskiDiff(c1, c2, dir);
        if (V * dir < 0) { // false return
            Simplex s = triangle.vertices;
            return std::make_pair(false, s);
        }
        triangle.vertices.push_front(V);
        if (triangle.handle(&dir)) { // true return
            Simplex s = triangle.vertices;
            return std::make_pair(true, s);
        }
    }
}

Vec3 CollisionHandler::normTo(const Vec3 &A, const Vec3 &B, const Vec3 &O) {
    Vec3 AB = B - A;
    Vec3 AO = O - A;
    return Calculations::unit(Calculations::cross(Calculations::cross(AB, AO), AB));
}

float distToOrigin(const Vec3 &A, const Vec3 &B) {
    float S2 = std::fabs(A.x*B.y - A.y*B.x);
    return S2 / (A-B).abs();
}

EPAResult CollisionHandler::EPA(ColliderPair pair, Simplex& s) {
    typedef std::pair<Vec3, Vec3> Edge;
    typedef struct VE {
        Vec3 vertex;
        float angle;
        float distToOrigin;
    } VE;
    typedef struct Polygon {
        std::list<VE> edges;
        static VE makeEdge(Vec3 &V, Vec3 &next) {
            return {
               V,
               std::atan2f(V.y, V.x) + m_pi,
               distToOrigin(V, next)
            };
        }
        void init(Simplex &s) {
            s.sort([](const Vec3 &a, const Vec3 &b) -> bool {
                return std::atan2f(a.y, a.x) < std::atan2f(b.y, b.x);
            });
            edges.clear();
            for (auto it = s.begin(); it != s.end(); it++) {
                auto next = std::next(it);
                if (next == s.end()) next = s.begin();
                    edges.push_back(makeEdge(*it, *next));
            }
        }
        Edge getNearest() {
            int no = (int)Calculations::argmax(edges, [](const VE& e1, const VE& e2) -> bool {
                return e1.distToOrigin > e2.distToOrigin;
            }); //TODO: cache this argmax
            auto it = edges.begin();
            auto first = std::next(it, no);
            auto second = std::next(first);
            if (second == edges.end()) second = edges.begin();
            return std::make_pair(first->vertex, second->vertex);
        }
        // TODO: maintain covexity on some cases (push when covering previous vertex)
        // may be check new vertex is outside of the neighbor edges.
        void push(Vec3 &V) {
            float angle = std::atan2f(V.y, V.x) + m_pi;
            auto first = edges.begin();
            auto second = std::prev(edges.end());
            auto insert = second;
            for (; first != edges.end(); first++) {
                second = std::next(first);
                if (second == edges.end()) {
                    if (angle > edges.front().angle) insert = edges.end();
                    else insert = edges.begin();
                    second = edges.begin();
                    break;
                }
                if ((angle > first->angle) && (angle < second->angle)) {
                    insert = second;
                    break;
                }
            }
            edges.insert(insert, {
                    V,
                    angle,
                    distToOrigin(V, second->vertex)
            });
            // update the it-V edge
            first->distToOrigin = distToOrigin(first->vertex, V);
        }
    } Polygon;
    Collider *c1 = pair.first;
    Collider *c2 = pair.second;
    Vec3 V, dir;
    Polygon polygon;
    Edge nearest;
    polygon.init(s);
    float last;
    while (true) {
        nearest = polygon.getNearest();
        dir = normTo(nearest.first, nearest.second, {0}) * -1;
        float now = distToOrigin(nearest.first, nearest.second);
        if (float_equlas(now, 0))
            return std::make_pair(0, Vec3{0});
        V = getMinkowskiDiff(c1, c2, dir);
        last = V * dir;
        //terminates;
        if (last < now)
            break;
        if (V * dir < 0)
            break;
        if (V == nearest.first || V == nearest.second)
            break;

        Vec3 v0 = nearest.first - nearest.second;
        Vec3 v1 = V - nearest.first;
        Vec3 v2 = V - nearest.second;
        if (v1.abs() + v2.abs() - v0.abs()< 0.001)
            break;

        polygon.push(V);
    }
    Vec3 normal = normTo(nearest.first, nearest.second, {0});
    renderer->addDebugInfo("Num Edges", polygon.edges.size());
    return std::make_pair(last, normal);
}

void PhysicsEngine::applyGravity() {
    Vec2 gravityForce = {0, gravity};
    for (RigidBody* object: rigid_bodies) {
        object->addForce(gravityForce * object->getMass());
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
        GJKResult gjkResult = handler.GJK(pair);
        if (gjkResult.first) {
            fillcircle(20, 20, 10);
            EPAResult epaResult = handler.EPA(pair, gjkResult.second);
            std::stringstream ss;
            renderer->addDebugInfo("Depth", epaResult.first);
        }
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

void PhysicsEngine::setRenderer(Renderer &r) {
    renderer = handler.renderer = &r;
}
