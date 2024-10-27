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

Vec3 Calculations::unit(const Vec3 &v) {
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

Vec3 Calculations::cross(const Vec3 &a, const Vec3 &b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vec3 Calculations::normTo(const Vec3 &A, const Vec3 &B, const Vec3 &O) {
    Vec3 AB = B - A;
    Vec3 AO = O - A;
    return Calculations::unit(Calculations::cross(Calculations::cross(AB, AO), AB));
}

float Calculations::dot(const Vector &a, const Vector &b) {
    float sum = 0;
    if (a.size() != b.size())
        throw std::runtime_error("unable to multiply vector of different sizes");
    for (int i = 0; i < a.size(); i++)
        sum += a[i] * b[i];
    return sum;
}

void Calculations::prodByElement(const Vector &a, const Vector &b, Vector &dest) {
    if (a.size() != b.size())
        throw std::runtime_error("unable to multiply vector of different sizes");
    std::transform(a.begin(), a.end(), b.begin(), dest.begin(), std::multiplies<float>());
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

Vec3 Vec2::unsqueeze(float z) const{
    return {x, y, z};
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

float Vec3::abs() const {
    return std::sqrtf(x*x + y*y + z*z);
}

Vec3 &Vec3::operator=(const Vec3 &other) = default;

bool Vec3::operator==(const Vec3 &other) const {
    return \
        float_equlas(x, other.x) && \
        float_equlas(y, other.y) && \
        float_equlas(z, other.z);
}


void Mat3::zero() {
    row1.zero();
    row2.zero();
    row3.zero();
}

Mat3 Mat3::transpose() const {
    return {
            {row1.x, row2.x, row3.x},
            {row1.y, row2.y, row3.y},
            {row1.z, row2.z, row3.z},
    };
}

Mat3 Mat3::identity() {
    return {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
    };
}

Mat3 &Mat3::operator=(const Mat3 &other) = default;

Vec3 operator+ (const Vec3 &a, const Vec3 &b) {
    return {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

Vec3 operator- (const Vec3 &a, const Vec3 &b) {
    return {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}

float operator* (const Vec3 &a, const Vec3 &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 operator* (const Vec3 &a, float b) {
    return {
        a.x * b,
        a.y * b,
        a.z * b
    };
}

Vec3 operator/ (const Vec3 &a, float b) {
    return {
        a.x / b,
        a.y / b,
        a.z / b
    };
}

Vec3 operator* (const Mat3 &m, const Vec3 &b) {
    return {
        m.row1 * b,
        m.row2 * b,
        m.row3 * b
    };
}

Mat3 operator* (const Mat3 &m, const Mat3 &n) {
    Mat3 t = n.transpose();
    Mat3 rt = {
            m * t.row1,
            m * t.row2,
            m * t.row3
    };
    return rt.transpose();
}

Mat3 getPose(const Mat2 &rot, const Vec3 &p) {
    return {
            rot.row1.unsqueeze(p.x),
            rot.row2.unsqueeze(p.y),
            {0, 0, 1}
    };
}

Mat3 getPose(const Vec3 &p) {
    Mat2 rot = Calculations::angle2RotMat(p.z);
    return {
            {rot.row1.x, rot.row1.y, p.x},
            {rot.row2.x, rot.row2.y, p.y},
            {0, 0, 1}
    };
}

Mat3 getInversePose(const Mat3 &p) {
    return Mat3 {
            {p.row1.x, p.row2.x, 0},
            {p.row1.y, p.row2.y, 0},
            {0, 0, 1}
    } * Mat3 {
            {1, 0, -p.row1.z},
            {0, 1, -p.row2.z},
            {0, 0, 1}
    };
}
