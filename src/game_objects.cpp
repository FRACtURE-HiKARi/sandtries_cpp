//
// Created by 13364 on 2024/10/15.
//

#include "game_objects.h"

void drawAABB(AABB *aabb) {
    rectangle(
            (int)aabb->lower_left.x,
            (int)aabb->upper_right.y,
            (int)aabb->upper_right.x,
            (int)aabb->lower_left.y
    );
}
TestObj::TestObj(float r) {
    this->r = r;
    addCollider(new BallCollider(1,r ));
}

void TestObj::show() {
    auto* c = (BallCollider*)collider_list.front();
    //AABB* aabb = c->getAABB();
    POINT points[4];
    circle((long)global_centroid().x, (long)global_centroid().y, (long)r);
    polygon(points, 4);
    //drawAABB(aabb);
}

Vec3 TestObj::pos() {
    return RigidBody::global_centroid();
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
    auto c = (RectangleCollider*)collider_list.front();
    //drawAABB(c->getAABB());
    POINT ps[4] = {
            {(long)c->vs[0].x, (long)c->vs[0].y},
            {(long)c->vs[1].x, (long)c->vs[1].y},
            {(long)c->vs[2].x, (long)c->vs[2].y},
            {(long)c->vs[3].x, (long)c->vs[3].y},
    };
    polygon(ps, 4);
}

void TestPolygon::show() {
    auto c = (PolygonCollider*)collider_list.front();
    size_t size = c->getSize();
    drawAABB(c->getAABB());
    std::vector<POINT> ps(size);
    for (int i = 0; i < size; i++)
        ps[i] = {(long) c->vs[i].x, (long) c->vs[i].y};
    polygon(ps.data(), (int)size);
}

TestRec::TestRec(int w, int h) {
    this->w = w;
    this->h = h;
    addCollider(new RectangleCollider(1, w, h));
}

void TestRec::show() {
    auto c = (RectangleCollider*)collider_list.front();
    std::vector<POINT> ps(4);
    for (int i = 0; i < 4; i++)
        ps[i] = {(long) c->vs[i].x, (long) c->vs[i].y};
    polygon(ps.data(), 4);
}
