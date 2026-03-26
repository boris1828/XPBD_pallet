#pragma once

#include <array>
#include <algorithm>

#include "types.h"
#include "AABB.cpp"


struct RigidBox;
struct TetraObject;
struct Cloth;

std::array<Real3, 4> get_tetra_points(TetraObject* obj, VertexIndex vs[4]);
std::array<Real3, 4> get_old_tetra_points(TetraObject* obj, VertexIndex vs[4]);

struct Constraint {
    Real compliance;
    Real lambda = 0.0;

    virtual ~Constraint() = default;

    Constraint(Real compliance) : compliance(compliance) {}

    void reset() { lambda = 0.0; }
};

// ##############################################

struct ClothEdge : Constraint {
    Cloth *cloth;
    VertexIndex v1, v2; 
    Real rest_length;

    ClothEdge(
        Real compliance, 
        Cloth *cloth,
        VertexIndex v1, 
        VertexIndex v2, 
        Real length) 
        : Constraint(compliance), 
          cloth(cloth), 
          rest_length(length),  
          v1(v1), v2(v2)
        {}

    void setObject(Cloth* o) { cloth = o; }
};

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
    AABB        aabb;
    Real3       normals[4];
    Real3       edges[6];
    Real3       center;
    Real3       old_center;
    bool        initialized;

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

    inline void reset() { initialized = false; }

    inline void init_normals_edges() {

        if (initialized) return;

        std::array<Real3, 4> ps     = get_tetra_points(obj, vs);
        std::array<Real3, 4> old_ps = get_old_tetra_points(obj, vs);

        auto addNormal = [&](Real3 a, Real3 b, Real3 c, Real3 opp, int idx) {
            Real3 normal     = glm::normalize(glm::cross(b - a, c - a));
            Real3 center     = (a + b + c) / 3.0;
            Real3 toOpposite = opp - center;
            if (glm::dot(normal, toOpposite) > 0.0) normal = -normal;

            normals[idx] = normal;
        };

        addNormal(ps[0], ps[1], ps[2], ps[3], 0);
        addNormal(ps[0], ps[2], ps[3], ps[1], 1);
        addNormal(ps[0], ps[1], ps[3], ps[2], 2);
        addNormal(ps[1], ps[2], ps[3], ps[0], 3);

        edges[0] = ps[1] - ps[0];
        edges[1] = ps[2] - ps[0];
        edges[2] = ps[3] - ps[0];
        edges[3] = ps[2] - ps[1];
        edges[4] = ps[3] - ps[1];
        edges[5] = ps[3] - ps[2];

        center     = (ps[0] + ps[1] + ps[2] + ps[3]) / 4.0;
        old_center = (old_ps[0] + old_ps[1] + old_ps[2] + old_ps[3]) / 4.0;

        initialized = true;
    }

    void update_aabb() {
        std::array<Real3, 4> positions = get_tetra_points(obj, vs);
        aabb = AABB(positions[0], positions[0]);
        for (int i=1; i<4; i++) aabb.expand(positions[i]);
    }
};

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
          v1(v1), v2(v2) {}
};

struct CollisionConstraint : InternalConstraint{
    VertexIndex v; 
    Real3 goal_position;
    Real3 normal;
    Real  penetration;
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

struct FixedRigidSpringConstraint : GlobalConstraint {

    RigidBox *box;
    Real3     body_attach;
    Real3     world_attach;
    Real      rest_length;

    FixedRigidSpringConstraint(
        Real compliance,
        RigidBox *box,
        Real3     body_attach,
        Real3     world_attach,
        Real      length)
        : GlobalConstraint(compliance),
          box(box),
          body_attach(body_attach),
          world_attach(world_attach),
          rest_length(length) {}
};

struct RigidSpringConstraint : GlobalConstraint {

    RigidBox *b1, *b2;
    Real3     r1, r2;
    Real      rest_length;
    bool active = true;

    RigidSpringConstraint(
        Real compliance,
        RigidBox *b1,
        RigidBox *b2,
        Real3     r1,
        Real3     r2,
        Real      length)
        : GlobalConstraint(compliance),
          b1(b1),
          b2(b2),
          r1(r1),
          r2(r2),
          rest_length(length) {}
};

struct RigidCollisionConstraint : GlobalConstraint {

    RigidBox *b1, *b2;
    Real3     p1, p2;
    Real3     r1, r2;
    Real      d;
    Real3     n;

    RigidCollisionConstraint(
        Real compliance,
        RigidBox *b1,
        RigidBox *b2,
        Real3     p1,
        Real3     p2,
        Real      penetration,
        Real3     normal)
        : GlobalConstraint(compliance),
          b1(b1),
          b2(b2),
          p1(p1),
          p2(p2),
          d(penetration),
          n(normal) {}
};

