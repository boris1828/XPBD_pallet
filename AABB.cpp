#include "types.cpp"

struct AABB {
    Real3 min;
    Real3 max;

    AABB() : min(Real3(0.0)), max(Real3(0.0)) {}
    AABB(Real3 _min, Real3 _max) : min(_min), max(_max) {}

    bool contains(const Real3 &point) const {
        return (point.x >= min.x && point.x <= max.x &&
                point.y >= min.y && point.y <= max.y &&
                point.z >= min.z && point.z <= max.z);
    }

    bool intersects(const AABB &other) const {
        return (min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y &&
                min.z <= other.max.z && max.z >= other.min.z);
    }
};