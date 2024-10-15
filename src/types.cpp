#include "types.h"
#include <cmath>

Vec2 operator+ (const Vec2 &a, const Vec2 &b) {
    return {
        a.x + b.x,
        a.y + b.y
    };
}

Vec2 operator- (const Vec2 &a, const Vec2 &b) {
    return {
        a.x - b.x,
        a.y - b.y
    };
}

float operator* (const Vec2 &a, const Vec2 &b) {
    return a.x * b.x + a.y * b.y;
}

Vec2 operator* (const Vec2 &a, const float b) {
    return {
        a.x * b,
        a.y * b
    };
}

Vec2& Vec2::operator= (const Vec2& other) = default;

Vec2 operator/ (const Vec2 &a, const float b) {
    return {
        a.x / b,
        a.y / b
    };
}

Vec2 operator* (const Mat2 &m, const Vec2 &b) {
    return {
        m.row1 * b,
        m.row2 * b
    };
}

// TODO: Fast Matmul operation (7steps)
Mat2 operator* (const Mat2 &m, const Mat2 &n) {
    Mat2 other = n.transpose();
    Mat2 result_transposed = {
        m * other.row1,
        m * other.row2
    };
    return result_transposed.transpose();
}

float Calculations::dot(const Vec2 &a, const Vec2 &b) {
    return a*b;
}

Mat2 Calculations::angle2RotMat(const float t) {
    return {
        {std::cosf(t), -std::sinf(t)},
        {std::sinf(t), std::cosf(t)}
    };
}

float Calculations::rotMat2Angle(const Mat2& m) {
    // TODO: assert errors when m is not a rotmat
    return std::acosf(m.row1.x);
}

float Calculations::cross(const Vec2 &a, const Vec2 &b) {
    return a.x * b.y - a.y * b.x;
}

Mat2 Mat2::transpose() const{
    return {
        {row1.x, row2.x},
        {row1.y, row2.y}
    };
}


void Vec2::zero() {
    x = y = 0;
}

float Vec2::abs() const {
    return std::sqrtf(x*x + y*y);
}


void Mat2::zero() {
    row1.zero();
    row2.zero();
}

Mat2 &Mat2::operator=(const Mat2 &other) = default;

std::ostream& operator<< (std::ostream& out, const Vec2 &v) {
    out << '(' << v.x << ", " << v.y << ')';
    return out;
}

std::ostream& operator<< (std::ostream& out, const Mat2 &m) {
    out << m.row1 << '\n' << m.row2;
    return out;
}
