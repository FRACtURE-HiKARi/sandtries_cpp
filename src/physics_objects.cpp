#include "physics_objects.h"
#include "utils.h"

Vec2 Collider::getCentroidOffset() const {
    return local_centroid * mass;
}

float Collider::getMass() const { return mass; }

// TODO: mR^2 + I0
float Collider::getInertia(const Vec2& host_centroid) const {
    Vec2 offset = host_centroid - local_centroid;
    return m_inertia + (offset * offset) * mass;
}

Collider::Collider(float mass) : PhysicsObject() {
    this->mass = mass;
    this->local_centroid = {0};
}

void Collider::initInertia() {
    throw std::runtime_error(
            "not expected to running into this function."
    );
}

void BallCollider::initInertia() {
    m_inertia = 0.5f * mass * radius * radius;
}

BallCollider::BallCollider(float mass, float radius): Collider(mass) {
    this->radius = radius;
    BallCollider::initInertia();
}

void RectangleCollider::initInertia() {
    m_inertia = mass * (h*h + w*w) / 12.0f;
}

RectangleCollider::RectangleCollider(float mass, float width, float height) \
: Collider(mass) {
    w = width;
    h = height;
    RectangleCollider::initInertia();
}

Vec2 PhysicsBody::globalToLocalVec(const Vec2 &p) {
    return rotation.transpose() * (p - global_centroid);
}

Vec2 PhysicsBody::localToGlobalVec(const Vec2 &p) {
    return rotation * (p + global_centroid);
}

void PhysicsBody::addCollider(Collider &collider) {
    collider_list.push_back(collider);
    local_centroid = local_centroid * mass + collider.getCentroidOffset();
    mass += collider.getMass();
    local_centroid = local_centroid / mass;

    m_inertia += collider.getInertia(local_centroid);
}

void PhysicsBody::zeroAll() {
    global_centroid.zero();
    local_centroid.zero();
    rotation.zero();
}

void StaticBody::addForce(const Vec2& force) {

}

void RigidBody::zeroAll() {
    PhysicsBody::zeroAll();
    velocity.zero();
    acceleration.zero();
    force.zero();
    torque = angular_acceleration = angular_velocity = 0;
}

// TODO: handle next tick collision conflict
Vec2 RigidBody::integratePosition(float dt) {
    acceleration = force / mass;
    velocity = velocity + acceleration * dt;
    global_centroid = global_centroid + velocity * dt;
    return global_centroid;
}

Mat2 RigidBody::integrateRotation(float dt) {
    angular_acceleration = torque / m_inertia;
    angular_velocity += angular_acceleration * dt;
    Mat2 rot_mat = Calculations::angle2RotMat(angular_velocity * dt);
    rotation = rot_mat * rotation;
    return rotation;
}

Pose RigidBody::integration(float dt){
    return {
        integratePosition(dt),
        integrateRotation(dt)
    };
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
