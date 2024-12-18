#pragma once
#include "types.h"
#include "nodes.h"
#include "utils.h"

struct AABB;
typedef std::list<AABB*> AABBList;

class PhysicsObject: public Node {
protected:
    Vec3 local_centroid{0};
    Mat3 pose_mat{};
    Mat3 pose_mat_inv{};
public:
    float mass;
    float m_inertia;
    float mass_inverse;
    float m_inertia_inverse;
    PhysicsObject();
    Vec3 global_centroid() const;
    Mat2 rotation() const;
    virtual void setPosition(const Vec2 &p);
    void setRotation(const Mat2 &r);
    Vec3 globalToLocalVec(const Vec3 &p);
    Vec3 localToGlobalVec(const Vec3 &p);
};

class PhysicsBody;
// describes the geometric shape of colliding objects.
class Collider: public PhysicsObject {
    // TODO: solve relative offset to the host body.
protected:
    AABB *aabb = nullptr;
    AABB *baseAABB = nullptr;
    virtual void initInertia() = 0;
    virtual void initAABB() = 0;
public:
    float restitution = 0.5;
    float friction = 0.1;
    PhysicsBody* body = nullptr;
    bool round_shaped = false;
    explicit Collider(float mass);
    Vec3 getCentroidOffset() const;
    AABB* getAABB();
    void setPosition(const Vec2 &p) override;
    virtual void setPose(const Mat3& pose);
    virtual void updateAABB() = 0;
    float getInertia(const Vec3 &host_centroid) const;
    virtual bool testRay(const Ray2 &ray, float *t, Vec2 *normal) = 0;
    virtual Vec3 supportVec(const Vec3& direction) const = 0;
    virtual float getEffectiveMass(Vec3 r);
    virtual ~Collider();
};

typedef std::list<Collider*> ColliderList;

class BallCollider: public Collider {
protected:
    void initInertia() override;
    void initAABB() override;
public:
    explicit BallCollider(float mass, float radius);
    bool testRay(const Ray2 &ray, float *t, Vec2 *normal) override;
    void setPosition(const Vec2 &p) override;
    Vec3 supportVec(const Vec3& direction) const override;
    void setPose(const Mat3 &pose) override;
    void updateAABB() override;
    float radius;
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
    Vec3 supportVec(const Vec3& direction) const override;
    void setPose(const Mat3 &pose) override;
    void updateAABB() override;
    std::array<Vec3, 4> vertices{};
    std::array<Vec3, 4> vs{};
};

// TODO: fill this collider
class PolygonCollider: public Collider {
protected:
    template <typename... Vec>
    void storeVertices(const Vec2 &v, Vec... vertices) {
        this->vertices.push_back(v.unsqueeze(1));
        storeVertices(vertices...);
    }
    void storeVertices() {}
    size_t size;
    void initInertia() override;
    void initAABB() override;
public:
    template <typename... Vec>
    explicit PolygonCollider(const Vec2 &vertex, Vec... vertices): Collider(1) {
        size = sizeof...(vertices) + 1;
        this->vertices.reserve(size);
        vs.assign(size, {});
        storeVertices(vertex, vertices...);
        std::sort(this->vertices.begin(), this->vertices.end(), [](const Vec3 &a, const Vec3 &b)->bool{
            return std::atan2f(a.y, a.x) < std::atan2f(b.y, b.x);
        });
    }
    size_t getSize() const {return size;}
    bool testRay(const Ray2 &ray, float *t, Vec2 *normal) override;
    void setPosition(const Vec2& p) override;
    Vec3 supportVec(const Vec3& direction) const override;
    void setPose(const Mat3 &pose) override;
    void updateAABB() override;
    std::vector<Vec3> vertices;
    std::vector<Vec3> vs;
};

class PhysicsBody: public PhysicsObject {
protected:
    ColliderList collider_list;
    void updateColliderPos();
public:
    //PhysicsBody();
    ~PhysicsBody();
    // TODO: solve place offset when adding
    void addCollider(Collider *collider);
    virtual void addForce(const Vec2 &f) = 0;
    virtual Vec3 velocityVector() = 0;
    virtual void addImpulse(const Vec3 &dv) = 0;
    void pushAllAABBs(AABBList & list);
    void setPosition(const Vec2 &p) override;
};

class StaticBody: public PhysicsBody {

public:
    void addForce(const Vec2& force) override;
    Vec3 velocityVector() override;
    void addImpulse(const Vec3 &dv) override;
};

class RigidBody: public PhysicsBody {
    Vec3 force_n_torque{};
    Vec3 acceleration{};
    Vec3 velocity{};

public:
    // TODO: a useful constructor
    RigidBody();
    void integration(float dt);
    void addForce(const Vec2 &force) override;
    void addTorque(float torque);
    void addTorque(Vec2 force, Vec2 offset);
    Vec3 velocityVector() override;
    void addImpulse(const Vec3 &dv) override;
    void clear();
};

inline AABB operator+ (const AABB& aabb, const Vec2& v);
inline AABB operator* (const Mat2& r, const AABB& aabb);

typedef struct AABB {
    Vec2 lower_left;
    Vec2 upper_right;
    Collider* collider;
    inline AABB& operator= (const AABB&) = default;
    inline AABB& operator+= (const Vec2& v);
    AABB(const AABB&) = default;
    bool contains(const Vec2& v) const;
    static bool meets(const AABB* a, const AABB* b);
    bool testRay(const Ray2 &ray) const;
    float distTo(const Vec2 &p) const;
} AABB;
