#include "physics_objects.h"
#include "utils.h"

PhysicsObject::PhysicsObject() {
    m_inertia = mass = mass_inverse = m_inertia_inverse = 0;
    local_centroid = {0};
    rotation = Calculations::angle2RotMat(0);
}

Vec2 Collider::getCentroidOffset() const {
    return local_centroid * mass;
}

float Collider::getMass() const { return mass; }

float Collider::getInertia(const Vec2& host_centroid) const {
    Vec2 offset = host_centroid - local_centroid;
    return m_inertia + (offset * offset) * mass;
}

Collider::Collider(float mass) : PhysicsObject() {
    this->mass = mass;
    this->local_centroid = {0};
    aabb = new AABB{0};
    aabb->collider = this;
}

void Collider::initInertia() {
    throw std::runtime_error(
            "not expected to running into this function."
    );
}

void Collider::setPosition(const Vec2 &p) {
    PhysicsObject::setPosition(p);
    *aabb = *baseAABB + p;
}

AABB *Collider::getAABB() {
    return aabb;
}

Collider::~Collider() {
    delete aabb;
    delete baseAABB;
}

void Collider::setPose(const Pose &pose) {
    rotation = pose.rotation;
    setPosition(pose.position);
}

void BallCollider::initInertia() {
    m_inertia = 0.5f * mass * radius * radius;
}

BallCollider::BallCollider(float mass, float radius): Collider(mass) {
    this->radius = radius;
    BallCollider::initInertia();
    BallCollider::initAABB();
}

bool BallCollider::testRay(const Ray2 &ray, float *t, Vec2 *normal) {
    Vec2 delta = global_centroid - ray.start;
    Vec2 proj = Calculations::proj(delta, ray.direction);
    if (proj * ray.direction < 0) return false;
    Vec2 nearest = ray.start + proj;
    float dist_squared = (global_centroid - nearest) * (global_centroid - nearest);
    if (dist_squared <= radius * radius) {
        *t = proj.abs() - std::sqrt(radius * radius - dist_squared);
        Vec2 hit = ray.start + ray.direction * *t;
        *normal = (hit - global_centroid);
        *normal = *normal / normal->abs();
        return true;
    }
    return false;
}

void BallCollider::initAABB() {
    baseAABB = new AABB{
            {-radius, -radius},
            {radius, radius},
            this
    };
}

Vec2 BallCollider::supportVec(const Vec2 &direction) const {
    return global_centroid + direction * radius;
}

void RectangleCollider::initInertia() {
    m_inertia = mass * (h*h + w*w) / 12.0f;
}

RectangleCollider::RectangleCollider(float mass, float width, float height) \
: Collider(mass) {
    w = width;
    h = height;
    vertices = {{
                        {-w / 2, -h / 2},
                        {w / 2, -h / 2},
                        {w / 2, h / 2},
                        {-w / 2, h / 2},
                }};
    RectangleCollider::initInertia();
    RectangleCollider::initAABB();
}

bool RectangleCollider::testRay(const Ray2 &ray, float *t, Vec2 *normal) {
    if (aabb->testRay(ray)) {
        // TODO: calculate hit point
        return true;
    }
    return false;
}

void RectangleCollider::initAABB() {
    baseAABB = new AABB{
            {-w/2, -h/2},
            {w/2, h/2},
            this
    };
}

void RectangleCollider::setPosition(const Vec2 &p) {
    Collider::setPosition(p);
    for (int i = 0; i < 4; i++)
        vs[i] = rotation*vertices[i] + global_centroid;
    float x, X, y, Y;
    x = X = vs[0].x;
    y = Y = vs[0].y;
    for (Vec2& v: vs) {
        if (x > v.x) x = v.x;
        if (X < v.x) X = v.x;
        if (y > v.y) y = v.y;
        if (Y < v.y) Y = v.y;
    }
    *aabb = {
            {x, y},
            {X, Y},
            this
    };
}

Vec2 RectangleCollider::supportVec(const Vec2 &direction) const {
    std::array<float, 4> dots{};
    for (int i = 0; i < 4; i++)
        dots[i] = (vs[i] - global_centroid) * direction;
    return vs[Calculations::argmax(dots)];
}

Vec2 PhysicsBody::globalToLocalVec(const Vec2 &p) {
    return rotation.transpose() * (p - global_centroid);
}

Vec2 PhysicsBody::localToGlobalVec(const Vec2 &p) {
    return rotation * (p + global_centroid);
}

void PhysicsBody::addCollider(Collider *collider) {
    collider_list.push_back(collider);
    local_centroid = local_centroid * mass + collider->getCentroidOffset();
    mass += collider->getMass();
    local_centroid = local_centroid / mass;

    m_inertia += collider->getInertia(local_centroid);

    mass_inverse = (mass == 0) ? 0 : 1/mass;
    m_inertia_inverse = (m_inertia == 0) ? 0 : 1/m_inertia;
}

PhysicsBody::~PhysicsBody() {
    for (Collider* collider: collider_list) {
        delete collider;
    }
}

void PhysicsBody::pushAllAABBs(AABBList &list) {
    for (Collider* collider: collider_list) {
        list.push_back(collider->getAABB());
    }
}

void PhysicsBody::updateColliderPos() {
    for (Collider* collider: collider_list) {
        collider->setPose({global_centroid, rotation});
    }
}

void PhysicsBody::setPosition(const Vec2 &p) {
    PhysicsObject::setPosition(p);
    updateColliderPos();
}

void StaticBody::addForce(const Vec2& force)
{/* do nothing */ }

RigidBody::RigidBody(): PhysicsBody(){
    velocity = {0};
    acceleration = {0};
    force = {0};
    torque = angular_acceleration = angular_velocity = 0;
}

Vec2 RigidBody::integratePosition(float dt) {
    acceleration = force * mass_inverse;
    velocity = velocity + acceleration * dt;
    global_centroid = global_centroid + velocity * dt;
    return global_centroid;
}

Mat2 RigidBody::integrateRotation(float dt) {
    angular_acceleration = torque * m_inertia_inverse;
    angular_velocity += angular_acceleration * dt;
    Mat2 rot_mat = Calculations::angle2RotMat(angular_velocity * dt);
    rotation = rot_mat * rotation;
    return rotation;
}

Pose RigidBody::integration(float dt){
    Pose result = {
        integratePosition(dt),
        integrateRotation(dt)
    };
    updateColliderPos();
    return result;
}

void RigidBody::addForce(const Vec2 &force) {
    this->force = this->force + force;
}

void RigidBody::addTorque(const float torque) {
    this->torque += torque;
}

void RigidBody::addTorque(Vec2 force, Vec2 offset) {
    addTorque(Calculations::cross(offset, force));
}

void RigidBody::clear() {
    force.zero();
    torque = 0;
}

inline AABB operator+ (const AABB& aabb, const Vec2& v) {
    return {
        aabb.lower_left + v,
        aabb.upper_right + v,
        aabb.collider
    };
}

inline AABB operator* (const Mat2& r, const AABB& aabb) {
    return {
        r * aabb.lower_left,
        r * aabb.upper_right
    };
}

bool AABB::contains(const Vec2 &v) const {
    {
        return
                v.x >= lower_left.x && \
                v.y >= lower_left.y && \
                v.x <= upper_right.x && \
                v.y <= upper_right.y;
    }
}

bool AABB::meets(const AABB *a, const AABB *b) {
    return
            a->contains(b->upper_right) || \
            a->contains(b->lower_left) || \
            a->contains({b->upper_right.x, b->lower_left.y}) || \
            a->contains({b->lower_left.x, b->upper_right.y}) || \
            b->contains(a->upper_right) || \
            b->contains(a->lower_left) || \
            b->contains({a->upper_right.x, a->lower_left.y}) || \
            b->contains({a->lower_left.x, a->upper_right.y}) \
        ;
}

bool AABB::testRay(const Ray2 &ray) const {
    std::vector<float> angles;
    angles.reserve(4);
    angles.push_back(std::atan2f(upper_right.y - ray.start.y, upper_right.x - ray.start.x));
    angles.push_back(std::atan2f(upper_right.y - ray.start.y, lower_left.x - ray.start.x));
    angles.push_back(std::atan2f(lower_left.y - ray.start.y, upper_right.x - ray.start.x));
    angles.push_back(std::atan2f(lower_left.y - ray.start.y, lower_left.x - ray.start.x));
    std::sort(angles.begin(), angles.end());
    float min = angles.at(0);
    float max = angles.at(3);
    float dir = std::atan2f(ray.direction.y, ray.direction.x);
    return (min <= dir) && (dir <= max);
}

float AABB::distTo(const Vec2 &p) const {
    float dx, dy;
    if (p.x < lower_left.x) dx = lower_left.x - p.x;
    else if (p.x > upper_right.x) dx = p.x - upper_right.x;
    else dx = 0;

    if (p.y < lower_left.y) dy = lower_left.y - p.y;
    else if (p.y > upper_right.y) dy = p.y - upper_right.y;
    else dy = 0;
    return std::sqrtf(dx*dx + dy*dy);
}
