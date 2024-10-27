/*
    Defines the types of data used in the physics engine.
*/

#pragma once
#include <iostream>
#include "utils.h"
#define FLOAT_ERROR 1e-5

typedef std::vector<float> Vector;

struct Vec3;
typedef struct Vec2 {
    float x;
    float y;
    void zero();
    float abs() const;
    Vec2& operator= (const Vec2 &other);
    bool operator== (const Vec2 &other) const;
    Vec3 unsqueeze(float z) const;
} Vec2;

typedef struct Vec3 {
    float x;
    float y;
    float z;
    void zero() {x = y = z = 0.f;}
    float abs() const;
    Vec3& operator= (const Vec3 &other);
    bool operator== (const Vec3 &other) const;
    Vec2 squeeze() const { return {x, y};}
} Vec3;

// TODO: overload operator[]
typedef struct Mat2 {
    Vec2 row1;
    Vec2 row2;
    void zero();
    Mat2& operator= (const Mat2& other);
    Mat2 transpose() const;
    Mat2 inv() const;
} Mat2;

typedef struct Mat3 {
    Vec3 row1;
    Vec3 row2;
    Vec3 row3;
    void zero();
    Mat3& operator= (const Mat3 &other);
    Mat3 transpose() const;
    static Mat3 identity();
} Mat3;

typedef struct Ray2 {
    Vec2 start;
    Vec2 direction;
} Ray2;

typedef struct Pose {
    Vec3 position;
    Mat2 rotation;
} Pose;

Vec2 operator+ (const Vec2 &a, const Vec2 &b);
Vec2 operator- (const Vec2 &a, const Vec2 &b);
float operator* (const Vec2 &a, const Vec2 &b);
Vec2 operator* (const Vec2 &a, float b);
Vec2 operator/ (const Vec2 &a, float b);

Vec3 operator+ (const Vec3 &a, const Vec3 &b);
Vec3 operator- (const Vec3 &a, const Vec3 &b);
float operator* (const Vec3 &a, const Vec3 &b);
Vec3 operator* (const Vec3 &a, float b);
Vec3 operator/ (const Vec3 &a, float b);

Vec2 operator* (const Mat2 &m, const Vec2 &b);
Mat2 operator* (const Mat2 &m, const Mat2 &n);

Vec3 operator* (const Mat3 &m, const Vec3 &b);
Mat3 operator* (const Mat3 &m, const Mat3 &n);

Mat3 getPose(const Mat2 &rot, const Vec3 &p);
Mat3 getPose(const Vec3 &p);
Mat3 getInversePose(const Mat3 &p);

class Calculations {
    public:
    static float dot(const Vec2 &a, const Vec2 &b);
    static float dot(const Vector &a, const Vector &b);
    static void prodByElement(const Vector &a, const Vector &b, Vector &dest);
    static float cross(const Vec2 &a, const Vec2 &b);
    static Mat2 angle2RotMat(float angle);
    static float rotMat2Angle(const Mat2& m);
    static Vec2 orth(const Vec2 &a);
    static Vec2 proj(const Vec2 &source, const Vec2 &target);
    static Vec2 unit(const Vec2 &v);
    static Vec2 normal(const Vec2 &source, const Vec2 &target);
    static float angle(const Vec2 &a, const Vec2 &b);
    static Vec2 orthDecompo(const Mat2 &sys, const Vec2& v);
    static Vec2 orthDecompo(const Vec2 &v0, const Vec2 &v1, const Vec2 &v2);
    static Vec3 cross(const Vec3 &a, const Vec3 &b);
    template<typename Container>
    static size_t argmax(const Container& c) {
        return std::distance(c.begin(), std::max_element(c.begin(), c.end()));
    }
    template<typename Container>
    static size_t argmin(const Container& c) {
        return std::distance(c.begin(), std::min_element(c.begin(), c.end()));
    }
    template <typename Container, typename Comparator>
    static size_t argmax(const Container& c, const Comparator& cmp) {
        return std::distance(c.begin(), std::max_element(c.begin(), c.end(), cmp));
    }
    static Vec3 normTo(const Vec3 &A, const Vec3 &B, const Vec3 &O);
    static Vec3 unit(const Vec3 &v);
};
inline bool float_equlas(const float a, const float b) {
    return std::abs(a - b) < FLOAT_ERROR;
}

static const float m_pi = std::acosf(-1);

std::ostream& operator<< (std::ostream& out, const Vec2 &v);
std::ostream& operator<< (std::ostream& out, const Mat2 &m);
