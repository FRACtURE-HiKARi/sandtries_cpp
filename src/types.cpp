#include "types.h"


Vec2 operator+ (const Vec2 &a, Vec2 &b) {
    return {
        a.x + b.x,
        a.y + b.y
    };
}

Vec2 operator- (const Vec2 &a, Vec2 &b) {
    return {
        a.x - b.x,
        a.y - b.y
    };
}


std::ostream& operator<< (std::ostream& out, const Vec2 &v) {
    out << '(' << v.x << ", " << v.y << ')';
    return out;
}
