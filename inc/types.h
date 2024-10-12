/*
    Defines the types of data used in the physics engine.
*/

#pragma once
#include <iostream>

typedef struct Vec2{
    float x;
    float y;
} Vec2;

Vec2 operator+ (const Vec2 &a, Vec2 &b);
Vec2 operator- (const Vec2 &a, Vec2 &b);
std::ostream& operator<< (std::ostream& out, const Vec2 &v);
