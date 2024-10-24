//
// Created by 13364 on 2024/10/15.
//

#include "game_objects.h"

TestObj::TestObj(float r) {
    this->r = r;
    addCollider(new BallCollider(1,r ));
}

void TestObj::show() {
    auto* c = (BallCollider*)collider_list.front();
    AABB* aabb = c->getAABB();
    POINT points[4];
    circle((long)global_centroid.x, (long)global_centroid.y, (long)r);
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
    RectangleCollider* c = (RectangleCollider*)collider_list.front();
    AABB *aabb = c->getAABB();
    rectangle(
            aabb->lower_left.x,
            aabb->upper_right.y,
            aabb->upper_right.x,
            aabb->lower_left.y
    );
    POINT ps[4] = {
            {(long)c->vs[0].x, (long)c->vs[0].y},
            {(long)c->vs[1].x, (long)c->vs[1].y},
            {(long)c->vs[2].x, (long)c->vs[2].y},
            {(long)c->vs[3].x, (long)c->vs[3].y},
    };
    polygon(ps, 4);
}
