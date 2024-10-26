#include "physics_objects.h"
#include "utils.h"

PhysicsObject::PhysicsObject() {
    m_inertia = mass = mass_inverse = m_inertia_inverse = 0;
    local_centroid = {0};
    pose_mat = Mat3::identity();
    pose_mat_inv = Mat3::identity();
}

Vec3 PhysicsObject::globalToLocalVec(const Vec3 &p) {
    return pose_mat_inv * p;
}

Vec3 PhysicsObject::localToGlobalVec(const Vec3 &p) {
    return pose_mat * p;
}

void PhysicsObject::setPosition(const Vec2 &p) {
    pose_mat.row1.z = p.x;
    pose_mat.row2.z = p.y;
    pose_mat_inv.row1.z = -p.x;
    pose_mat_inv.row2.z = -p.y;
}

Vec3 PhysicsObject::global_centroid() const {
    return {pose_mat.row1.z, pose_mat.row2.z, 1};
}

Mat2 PhysicsObject::rotation() const {
    return {
        pose_mat.row1.squeeze(),
        pose_mat.row2.squeeze()
    };
}

void PhysicsObject::setRotation(const Mat2 &r) {
    pose_mat.row1.x = r.row1.x;
    pose_mat.row1.y = r.row1.y;
    pose_mat.row2.x = r.row2.x;
    pose_mat.row2.y = r.row2.y;
    pose_mat_inv = getInversePose(pose_mat);
}

Vec3 Collider::getCentroidOffset() const {
    return local_centroid * mass;
}

float Collider::getInertia(const Vec3& host_centroid) const {
    Vec3 offset = host_centroid - local_centroid;
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
}

AABB *Collider::getAABB() {
    return aabb;
}

Collider::~Collider() {
    delete aabb;
    delete baseAABB;
}

void Collider::setPose(const Mat3& pose) {
    pose_mat = pose;
    pose_mat_inv = getInversePose(pose_mat);
}

void BallCollider::initInertia() {
    m_inertia = 0.5f * mass * radius * radius;
}

BallCollider::BallCollider(float mass, float radius): Collider(mass) {
    this->radius = radius;
    round_shaped = true;
    BallCollider::initInertia();
    BallCollider::initAABB();
}

bool BallCollider::testRay(const Ray2 &ray, float *t, Vec2 *normal) {
    Vec2 delta = global_centroid().squeeze() - ray.start;
    Vec2 proj = Calculations::proj(delta, ray.direction);
    if (proj * ray.direction < 0) return false;
    Vec2 nearest = ray.start + proj;
    float dist_squared = (global_centroid().squeeze() - nearest) * (global_centroid().squeeze() - nearest);
    if (dist_squared <= radius * radius) {
        *t = proj.abs() - std::sqrt(radius * radius - dist_squared);
        Vec2 hit = ray.start + ray.direction * *t;
        *normal = (hit - global_centroid().squeeze());
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

Vec3 BallCollider::supportVec(const Vec3 &direction) const {
    return global_centroid() + direction * radius;
}

void BallCollider::setPosition(const Vec2 &p) {
    Collider::setPosition(p);
    updateAABB();
}

void BallCollider::setPose(const Mat3 &pose) {
    Collider::setPose(pose);
    updateAABB();
}

void BallCollider::updateAABB() {
    *aabb = *baseAABB + global_centroid().squeeze();
}

void RectangleCollider::initInertia() {
    m_inertia = mass * (h*h + w*w) / 12.0f;
}

RectangleCollider::RectangleCollider(float mass, float width, float height) \
: Collider(mass) {
    w = width;
    h = height;
    vertices = {{
                        {-w / 2, -h / 2, 1},
                        {w / 2, -h / 2, 1},
                        {w / 2, h / 2, 1},
                        {-w / 2, h / 2, 1},
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
    updateAABB();
}

Vec3 RectangleCollider::supportVec(const Vec3 &direction) const {
    std::array<float, 4> dots{};
    for (int i = 0; i < 4; i++)
        dots[i] = (vs[i] - global_centroid()) * direction;
    return vs[Calculations::argmax(dots)];
}

void RectangleCollider::setPose(const Mat3 &pose) {
    Collider::setPose(pose);
    updateAABB();
}

void RectangleCollider::updateAABB() {
    for (int i = 0; i < 4; i++)
        vs[i] = localToGlobalVec(vertices[i]);
    float x, X, y, Y;
    x = X = vs[0].x;
    y = Y = vs[0].y;
    for (Vec3& v: vs) {
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

bool PolygonCollider::testRay(const Ray2 &ray, float *t, Vec2 *normal) {
    return false;
}

void PolygonCollider::setPosition(const Vec2 &p) {
    Collider::setPosition(p);
    updateAABB();
}

Vec3 PolygonCollider::supportVec(const Vec3 &direction) const {
    std::vector<float> dots(size);
    Vec3 gc = global_centroid();
    for (int i = 0; i < size; i++) {
        dots[i] = (vs[i] - gc) * direction;
    }
    return vs[Calculations::argmax(dots)];
}

void PolygonCollider::setPose(const Mat3 &pose) {
    Collider::setPose(pose);
    updateAABB();
}

void PolygonCollider::updateAABB() {
    for (int i = 0; i < size; i++)
        vs[i] = localToGlobalVec(vertices[i]);
    float x, X, y, Y;
    Vec3& front = vs[0];
    x = X = front.x;
    y = Y = front.y;
    for (int i = 1; i < size; i++) {
        Vec3 &v = vs[i];
        if (x > v.x) x = v.x;
        if (X < v.x) X = v.x;
        if (y > v.y) y = v.y;
        if (Y < v.y) Y = v.y;
    }
    aabb->lower_left = {x, y};
    aabb->upper_right = {X, Y};
}

void PolygonCollider::initInertia() {
    m_inertia = m_inertia_inverse = 0;
}

void PolygonCollider::initAABB() {
    aabb = new AABB{};
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
        collider->setPose(pose_mat);
    }
}

void PhysicsBody::setPosition(const Vec2 &p) {
    PhysicsObject::setPosition(p);
    updateColliderPos();
}

void StaticBody::addForce(const Vec2& force)
{/* do nothing */ }

RigidBody::RigidBody(): PhysicsBody(){

}

void RigidBody::integration(float dt){
    acceleration = {
            force_n_torque.x * mass_inverse,
            force_n_torque.y * mass_inverse,
            force_n_torque.z * m_inertia_inverse
    };
    velocity = velocity + acceleration * dt;
    Vec3 delta = velocity * dt;
    pose_mat = getPose({delta.x, delta.y, 0}) * pose_mat * getPose({0, 0, delta.z});
    pose_mat_inv = getInversePose(pose_mat);
    updateColliderPos();
}

void RigidBody::addForce(const Vec2 &force) {
    force_n_torque.x += force.x;
    force_n_torque.y += force.y;
}

void RigidBody::addTorque(const float torque) {
    force_n_torque.z += torque;
}

void RigidBody::addTorque(Vec2 force, Vec2 offset) {
    addTorque(Calculations::cross(offset, force));
}

void RigidBody::clear() {
    force_n_torque.zero();
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
            !(a->lower_left.x > b->upper_right.x || \
              b->lower_left.x > a->upper_right.x || \
              a->lower_left.y > b->upper_right.y || \
              b->lower_left.y > a->upper_right.y)
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

AABB &AABB::operator+=(const Vec2 &v)
{
    *this = *this + v;
    return *this;
}
