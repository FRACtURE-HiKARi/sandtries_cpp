//
// Created by 13364 on 2024/10/15.
//

#pragma once
#include "game_engine.h"

class TestObj: public Visible, public RigidBody {
    float r;
public:
    explicit TestObj(float r);
    void show() override;
    Vec3 pos();
    void setPos(const Vec2& pose);
};

class TestStatic: public Visible, public StaticBody {
    int w, h;
public:
    explicit TestStatic(int w, int h);
    void show() override;
    void setPos(const Vec2& pose);

};

class TestPolygon: public Visible, public StaticBody {
public:
    void show() override;
    void setPos(const Vec2 &p) {StaticBody::setPosition(p);}
};