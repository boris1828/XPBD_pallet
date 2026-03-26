#pragma once

#include "types.h"

struct AABB 
{
    Real3 min;
    Real3 max;

    AABB() : min(Real3(0.0)), max(Real3(0.0)) {}
    AABB(Real3 _min, Real3 _max) : min(_min), max(_max) {}

    void reset() 
    {
        min = Real3(0.0);
        max = Real3(0.0);
    }

    bool contains(const Real3 &point, Real epsilon = Real(0.0)) const 
    {
        return (point.x >= min.x - epsilon && point.x <= max.x + epsilon &&
                point.y >= min.y - epsilon && point.y <= max.y + epsilon &&
                point.z >= min.z - epsilon && point.z <= max.z + epsilon);
    }
    
    bool intersects(const AABB &other) const 
    {
        return (min.x <= other.max.x && max.x >= other.min.x &&
                min.y <= other.max.y && max.y >= other.min.y &&
                min.z <= other.max.z && max.z >= other.min.z);
    }

    void expand(const Real3 &point) 
    {
        min = glm::min(min, point);
        max = glm::max(max, point);
    }
};