/*
    Defines the types of data used in the physics engine.
*/

#pragma once
#include <iostream>

typedef struct Vec2 {
    float x;
    float y;
    void zero();
    float abs() const;
    Vec2& operator= (const Vec2& other);
} Vec2;

// TODO: overload operator[]
typedef struct Mat2 {
    Vec2 row1;
    Vec2 row2;
    void zero();
    Mat2& operator= (const Mat2& other);
    Mat2 transpose() const;
} Mat2;

typedef struct Pose {
    Vec2 position;
    Mat2 rotation;
} Pose;

Vec2 operator+ (const Vec2 &a, const Vec2 &b);
Vec2 operator- (const Vec2 &a, const Vec2 &b);
float operator* (const Vec2 &a, const Vec2 &b);
Vec2 operator* (const Vec2 &a, float b);
Vec2 operator/ (const Vec2 &a, float b);

Vec2 operator* (const Mat2 &m, const Vec2 &b);
Mat2 operator* (const Mat2 &m, const Mat2 &n);

class Calculations {
    public:
    static float dot(const Vec2 &a, const Vec2 &b);
    static float cross(const Vec2 &a, const Vec2 &b);
    static Mat2 angle2RotMat(float t);
    static float rotMat2Angle(const Mat2& m);
};

std::ostream& operator<< (std::ostream& out, const Vec2 &v);
std::ostream& operator<< (std::ostream& out, const Mat2 &m);
