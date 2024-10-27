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
            *d = Calculations::normTo(a, b, {0});
            return false;
        }
        bool triangleCase(Vec3 *d) {
            auto it = vertices.begin();
            Vec3 a = *(it++);
            Vec3 b = *(it++);
            Vec3 c = *it;
            Vec3 n_ab = Calculations::normTo(a, b, c);
            Vec3 n_ac = Calculations::normTo(a, c, b);
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
    if (c1->round_shaped && c2->round_shaped)
        return circles((BallCollider*)pair.first, (BallCollider*)pair.second);
    Vec3 V, dir;
    Polygon polygon;
    Edge nearest;
    polygon.init(s);
    float last;
    while (true) {
        nearest = polygon.getNearest();
        dir = Calculations::normTo(nearest.first, nearest.second, {0}) * -1;
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
    Vec3 normal = Calculations::normTo(nearest.first, nearest.second, {0});
    return std::make_pair(last, normal);
}

EPAResult CollisionHandler::circles(BallCollider* b1, BallCollider* b2) {
    Vec3 delta = b1->global_centroid() - b2->global_centroid();
    Vec3 dir = Calculations::unit(delta);
    float dist = delta.abs() - b1->radius - b2->radius;
    return std::make_pair(dist, dir);
}

void CollisionHandler::assignImpulse(ColliderPair pair, EPAResult epaResult, float dt) {
    Vec3 N = epaResult.second;
    Vec3 va = pair.first->body->velocityVector();
    Vec3 vb = pair.second->body->velocityVector();
    Vec3 wa = {0, 0, va.z};
    Vec3 wb = {0, 0, vb.z};
    va.z = vb.z = 0;
    float e = (pair.first->restitution < pair.second->restitution) ? \
                pair.first->restitution : pair.second->restitution;
    float miu = (pair.first->friction > pair.second->friction) ? \
                pair.first->friction : pair.second->friction;
    Ray2 ray{pair.first->global_centroid().squeeze(), N.squeeze() * -1};
    float t;
    Vec2 n;
    pair.first->testRay(ray, &t, &n);
    Vec3 ra = n.unsqueeze(0) * t * -1;
    ray = {pair.second->global_centroid().squeeze(), N.squeeze()};
    pair.second->testRay(ray, &t, &n);
    Vec3 rb = n.unsqueeze(0) * t * -1;
    Vec3 dv = va + Calculations::cross(wa, ra) - vb - Calculations::cross(wb, rb);
    float dv_rel = dv * N;
    if (dv_rel > 0) return;

    Vector V {va.x, va.y, wa.z, vb.x, vb.y, wb.z};
    Vector J {N.x, N.y, Calculations::cross(N, ra).z, -N.x, -N.y, Calculations::cross(rb, N).z};
    float ma_i = pair.first->body->mass_inverse;
    float ia_i = pair.first->body->m_inertia_inverse;
    float mb_i = pair.second->body->mass_inverse;
    float ib_i = pair.second->body->m_inertia_inverse;
    Vector M_inv {ma_i, ma_i, ia_i, mb_i, mb_i, ib_i};
    Vector tmp(6);
    Calculations::prodByElement(M_inv, J, tmp);
    float m_effective = Calculations::dot(tmp, J);
    float beta = 0.01;
    float d = (pair.first->global_centroid() - pair.second->global_centroid()) * N;
    float s_d = 10;
    float s_v = 5;
    d = (d > s_d) ? d - s_d : 0;
    dv_rel = (dv_rel > s_v) ? dv_rel - s_v : 0;
    float b = -beta * d / dt + dv_rel * e;
    float lambda = -(Calculations::dot(J, V) + b) / m_effective;
    renderer->addDebugInfo("d", d);
    renderer->addDebugInfo("b", b);
    renderer->addDebugInfo("l", lambda);
    std::for_each(tmp.begin(), tmp.end(), [&lambda](float &a){a *= lambda;});
    pair.first->body->addImpulse({tmp[0], tmp[1], tmp[2]});
    pair.second->body->addImpulse({tmp[3], tmp[4], tmp[5]});
    // friction
    /*
    Vec3 T = {N.y, -N.x, 0};
    J = {T.x, T.y, Calculations::cross(T, ra).z, -T.x, -T.y, Calculations::cross(rb, T).z};
    m_effective = Calculations::dot(tmp, J);
    Calculations::prodByElement(M_inv, J, tmp);
    dv_rel = dv * T;
    d = (pair.first->global_centroid() - pair.second->global_centroid()) * T;
    b = -beta * d / dt + dv_rel * miu;
    lambda = -(Calculations::dot(J, V) + b) / m_effective;
    std::for_each(tmp.begin(), tmp.end(), [&lambda](float &a){a *= lambda;});
    pair.first->body->addImpulse({tmp[0], tmp[1], tmp[2]});
    pair.second->body->addImpulse({tmp[3], tmp[4], tmp[5]});
    */
}

void PhysicsEngine::applyGravity() {
    Vec2 gravityForce = {0, gravity};
    for (RigidBody* object: rigid_bodies) {
        object->addForce(gravityForce * object->mass);
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
    }
    ColliderPairList candidates = broadPhase.getAllPairs();
    for (ColliderPair pair: candidates) {
        if ((std::find(static_bodies.begin(), static_bodies.end(), pair.first->body) != static_bodies.end()) \
            && (std::find(static_bodies.begin(), static_bodies.end(), pair.second->body) != static_bodies.end())
        ) continue;
        GJKResult gjkResult = handler.GJK(pair);
        if (gjkResult.first) {
            EPAResult epaResult = handler.EPA(pair, gjkResult.second);
            handler.assignImpulse(pair, epaResult, dt);
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
