#include "physics.h"

Vec2 PhysicsBody::globalToLocalVec(const Vec2 &p) {
    return p - global_centroid;
}


Vec2 PhysicsBody::localToGlobalVec(const Vec2 &p) {
    return p + global_centroid;
}
