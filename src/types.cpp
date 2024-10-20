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

Mat2 Calculations::angle2RotMat(const float angle) {
    return {
        {std::cosf(angle), -std::sinf(angle)},
        {std::sinf(angle), std::cosf(angle)}
    };
}

float Calculations::rotMat2Angle(const Mat2& m) {
    // TODO: assert errors when m is not a rotmat
    return std::acosf(m.row1.x);
}

float Calculations::cross(const Vec2 &a, const Vec2 &b) {
    return a.x * b.y - a.y * b.x;
}

Vec2 Calculations::orth(const Vec2 &a) {
    return Vec2{
        -a.y,
        a.x
    };
}

Vec2 Calculations::proj(const Vec2 &source, const Vec2 &target) {
    Vec2 result = target;
    result = result * (source * target);
    result = result / (target * target);
    return result;
}

Vec2 Calculations::unit(const Vec2 &v) {
    return v / v.abs();
}

Vec2 Calculations::normal(const Vec2 &source, const Vec2 &target) {
    return unit(source - proj(source, target));
}

float Calculations::angle(const Vec2 &a, const Vec2 &b) {
    return std::acosf(a * b / a.abs() / b.abs());
}

Vec2 Calculations::orthDecompo(const Mat2 &sys, const Vec2 &v) {
    return sys.inv() * v;
}

Vec2 Calculations::orthDecompo(const Vec2 &v0, const Vec2 &v1, const Vec2 &v2) {
    Mat2 sysT = {v0, v1};
    return orthDecompo(sysT.transpose(), v2);
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

bool Vec2::operator==(const Vec2 &other) const {
    return float_equlas(x, other.x) && float_equlas(y, other.y);
}


void Mat2::zero() {
    row1.zero();
    row2.zero();
}

Mat2 Mat2::inv() const {
    float det = this->row1.x * this->row2.y - this->row1.y * this->row2.x;
    if (det == 0) return {
                {NAN, NAN},
                {NAN, NAN}
    };
    return {
            Vec2{row2.y, -row2.x} / det,
            Vec2{-row1.y, row1.x} / det,
    };
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
