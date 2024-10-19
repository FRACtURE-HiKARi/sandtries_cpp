#pragma once
#include "types.h"
#include "nodes.h"
#include "utils.h"

struct AABB;
typedef std::list<AABB*> AABBList;

class PhysicsObject: public Node {
protected:
    float mass;
    float m_inertia;
    float mass_inverse;
    float m_inertia_inverse;
    Vec2 local_centroid{0};
    Vec2 global_centroid{0};
    Mat2 rotation{0};
public:
    PhysicsObject();
    Vec2 getPosition() const {return global_centroid;}
    virtual void setPosition(const Vec2 &p) {global_centroid = p;}
};

// describes the geometric shape of colliding objects.
class Collider: public PhysicsObject {
    // TODO: solve relative offset to the host body.
protected:
    Vec2 offset;
    AABB *aabb = nullptr;
    AABB *baseAABB = nullptr;
    virtual void initInertia() = 0;
    virtual void initAABB() = 0;
    //PhysicsBody* body;
public:
    explicit Collider(float mass);
    Vec2 getCentroidOffset() const;
    float getMass() const;
    AABB* getAABB();
    void setPosition(const Vec2 &p) override;
    void setPose(const Pose& pose);
    float getInertia(const Vec2 &host_centroid) const;
    virtual bool testRay(const Ray2 &ray, float *t, Vec2 *normal) = 0;
    virtual Vec2 supportVec(const Vec2& direction) const = 0;
    virtual ~Collider();
};

typedef std::list<Collider*> ColliderList;

class BallCollider: public Collider {
protected:
    float radius;
    void initInertia() override;
    void initAABB() override;
public:
    explicit BallCollider(float mass, float radius);
    bool testRay(const Ray2 &ray, float *t, Vec2 *normal) override;
    Vec2 supportVec(const Vec2& direction) const override;
};

class RectangleCollider: public Collider {
protected:
    float h;
    float w;

    void initInertia() override;
    void initAABB() override;
public:
    explicit RectangleCollider(float mass, float width, float height);
    bool testRay(const Ray2 &ray, float *t, Vec2 *normal) override;
    void setPosition(const Vec2& p) override;
    Vec2 supportVec(const Vec2& direction) const override;
    std::array<Vec2, 4> vertices{};
    std::array<Vec2, 4> vs{};
};

// TODO: fill this collider
class PolygonCollider: public Collider {

};

class PhysicsBody: public PhysicsObject {
protected:
    ColliderList collider_list;
    void updateColliderPos();
public:
    //PhysicsBody();
    ~PhysicsBody();
    Vec2 globalToLocalVec(const Vec2 &p);
    Vec2 localToGlobalVec(const Vec2 &p);
    // TODO: solve place offset when adding
    void addCollider(Collider *collider);
    virtual void addForce(const Vec2 &f) = 0;
    void pushAllAABBs(AABBList & list);
    void setPosition(const Vec2 &p) override;
};

class StaticBody: public PhysicsBody {

public:
    void addForce(const Vec2& force) override;
};

class RigidBody: public PhysicsBody {
    Vec2 velocity{};
    Vec2 acceleration{};
    Vec2 force{};
    float torque;
    float angular_velocity;
    float angular_acceleration;

    Vec2 integratePosition(float dt);
    Mat2 integrateRotation(float dt);

public:
    // TODO: a useful constructor
    RigidBody();
    Pose integration(float dt);
    void addForce(const Vec2 &force) override;
    void addTorque(float torque);
    void addTorque(Vec2 force, Vec2 offset);
    void clear();
};

inline AABB operator+ (const AABB& aabb, const Vec2& v);
inline AABB operator* (const Mat2& r, const AABB& aabb);

typedef struct AABB {
    Vec2 lower_left;
    Vec2 upper_right;
    Collider* collider;
    inline AABB& operator= (const AABB&) = default;
    inline AABB& operator+= (const Vec2& v) {
        *this = *this + v;
        return *this;
    }
    AABB(const AABB&) = default;
    bool contains(const Vec2& v) const;
    static bool meets(const AABB* a, const AABB* b);
    bool testRay(const Ray2 &ray) const;
    float distTo(const Vec2 &p) const;
} AABB;
