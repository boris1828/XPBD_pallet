#pragma once

#include <array>
#include <algorithm>

#include "types.cpp"

struct TetraObject;

std::array<Real3, 4> get_tetra_points(TetraObject* obj, VertexIndex vs[4]);

struct Constraint {
    Real compliance;
    Real lambda = 0.0;

    virtual ~Constraint() = default;

    Constraint(Real compliance) : compliance(compliance) {}

    void reset() { lambda = 0.0; }
};

// ##############################################

struct InternalConstraint : Constraint {
    TetraObject *obj;

    InternalConstraint(
        Real compliance, 
        TetraObject *o) 
        : Constraint(compliance), 
          obj(o) {}

    void setObject(TetraObject* o) { obj = o; }
};

struct GlobalConstraint : Constraint {

    GlobalConstraint(Real compliance) 
        : Constraint(compliance) {}
};

// ##############################################

struct Tetrahedron : InternalConstraint {
    VertexIndex vs[4];
    Real        rest_volume;
    Real3       center;
    Real        radius;

    Tetrahedron(
        Real compliance,
        TetraObject *obj,
        VertexIndex v1, 
        VertexIndex v2, 
        VertexIndex v3, 
        VertexIndex v4, 
        Real volume)
        : InternalConstraint(compliance, obj), 
          rest_volume(volume) 
           {
        vs[0] = v1;
        vs[1] = v2; 
        vs[2] = v3;
        vs[3] = v4;
    }

    void update_bounding_sphere() {
        std::array<Real3, 4> positions = get_tetra_points(obj, vs);
        Real3 p1 = positions[0];
        Real3 p2 = positions[1];
        Real3 p3 = positions[2];
        Real3 p4 = positions[3];
        center = (p1 + p2 + p3 + p4) / 4.0;
        radius = glm::length(p1 - center);
        radius = std::max(radius, glm::length(p2 - center));
        radius = std::max(radius, glm::length(p3 - center));
        radius = std::max(radius, glm::length(p4 - center));        
    }
};

bool bounding_sphere_intersect(const Tetrahedron& t1, const Tetrahedron& t2) {
    Real3 diff = { t1.center.x - t2.center.x,
                   t1.center.y - t2.center.y,
                   t1.center.z - t2.center.z };

    Real distSq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
    Real rSum   = t1.radius + t2.radius;

    return distSq <= rSum * rSum;
}

struct Edge : InternalConstraint {
    VertexIndex v1, v2; 
    Real rest_length;

    Edge(
        Real compliance,
        TetraObject *obj,
        VertexIndex v1, 
        VertexIndex v2, 
        Real length)
        : InternalConstraint(compliance, obj),
          rest_length(length),  
          v1(v1), 
          v2(v2) {}
};

struct CollisionConstraint : InternalConstraint{
    VertexIndex v; 
    Real3 goal_position;
    bool  active;

    CollisionConstraint()
        : InternalConstraint(0.0, nullptr), 
          v(0), 
          goal_position(Real3(0.0)), 
          active(false) {}

    CollisionConstraint(
        Real compliance,
        TetraObject *obj,
        VertexIndex v, 
        Real3 goal_position, 
        bool active = false)
        : v(v),
          goal_position(goal_position), 
          active(active), 
          InternalConstraint(compliance, obj) {}
};

// ##############################################

struct SpringConstraint : GlobalConstraint{
    TetraObject *obj1, *obj2;
    VertexIndex v1, v2;
    Real rest_length;

    SpringConstraint(
        Real compliance,
        TetraObject *obj1, 
        TetraObject *obj2, 
        VertexIndex v1, 
        VertexIndex v2, 
        Real length)
        : GlobalConstraint(compliance),
          obj1(obj1), 
          obj2(obj2), 
          v1(v1), 
          v2(v2), 
          rest_length(length) {}
};


