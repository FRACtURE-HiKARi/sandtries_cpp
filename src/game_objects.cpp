//
// Created by 13364 on 2024/10/15.
//

#include "game_objects.h"

TestObj::TestObj(float w, float h) {
    this->w = w;
    this->h = h;
    addCollider(new RectangleCollider(1, w, h));
}

void TestObj::show() {
    auto* c = (RectangleCollider*)collider_list.front();
    AABB* aabb = c->getAABB();
    POINT points[4];
    for (int i = 0; i < 4; i++)
        points[i] = {(long)(c->vs[i].x), (long)(c->vs[i].y)};
    polygon(points, 4);
    rectangle(
            aabb->lower_left.x,
            aabb->upper_right.y,
            aabb->upper_right.x,
            aabb->lower_left.y
    );
}

Vec2 TestObj::pos() {
    return RigidBody::global_centroid;
}

void TestObj::setPos(const Vec2 &pose) {
    RigidBody::setPosition(pose);
}

TestStatic::TestStatic(int w, int h): StaticBody() {
    addCollider(new RectangleCollider(1, (float)w, (float)h));
    this->w = w;
    this->h = h;
}

void TestStatic::setPos(const Vec2 &pose) {
    StaticBody::setPosition(pose);
}

void TestStatic::show() {
    AABB *aabb = collider_list.front()->getAABB();
    rectangle(
            aabb->lower_left.x,
            aabb->upper_right.y,
            aabb->upper_right.x,
            aabb->lower_left.y
    );
}
