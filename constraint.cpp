#pragma once

#include "types.cpp"

struct TetraObject;

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
        TetraObject *obj) 
        : Constraint(compliance), 
          obj(obj) {}

    void setObject(TetraObject* o) { obj = o; }
};

struct GlobalConstraint : Constraint {

    GlobalConstraint(Real compliance) 
        : Constraint(compliance) {}
};

// ##############################################

struct Tetrahedron : InternalConstraint {
    VertexIndex vs[4];
    Real rest_volume;

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


