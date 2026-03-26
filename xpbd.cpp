#pragma once

#include <vector>
#include <chrono>

#include "object.cpp"
#include "rigid.cpp"
#include "cloth.cpp"
#include "types.h"

#include "collision.cpp"
#include "settings.cpp"

#include <stdio.h>

static uint64_t frequency; //25*4*60;
static uint64_t iterations_per_step = 1;
static Real delta_t;

extern Real coll_compliance;
extern Real mu_dynamic;

Real3 gravity(0.0, -9.81, 0.0);

Real ground_y = -2.0;

struct Collision {
    TetraObject *obj1;
    TetraObject *obj2;
    TetraIndex t1;
    TetraIndex t2;
    CollisionInfo info;
    Real3 dp_tang;
};

void XPBD_init(uint64_t heartz = 1000, uint64_t iterations = 1) 
{
    frequency           = heartz;
    iterations_per_step = iterations;
    delta_t             = 1.0 / frequency;
}

void XPBD_collect_collisions(
        std::vector<TetraObject> &objects, 
        std::vector<Collision>   &collisions) 
{
    collisions.clear();

    for (auto& obj : objects) 
        for (size_t vi = 0; vi < obj.num_vertices(); vi++) 
            obj.vertex_collisions[vi].clear();

    size_t coll_idx = 0; 

    for (size_t o1i = 0; o1i < objects.size(); o1i++) {
        for (size_t o2i = o1i+1; o2i < objects.size(); o2i++) {

            if (!objects[o1i].aabb.intersects(objects[o2i].aabb)) continue; 

            for (size_t t1i = 0; t1i < objects[o1i].num_tetras(); t1i++) {
                for (size_t t2i = 0; t2i < objects[o2i].num_tetras(); t2i++) {

                    Tetrahedron &t1 = objects[o1i].tetras[t1i];
                    Tetrahedron &t2 = objects[o2i].tetras[t2i];

                    if (!t1.aabb.intersects(t2.aabb)) continue;

                    CollisionInfo info = SAT_tet_tet(
                                                objects[o1i], t1, 
                                                objects[o2i], t2);

                    if (!info.intersecting) continue;

                    Real3 dp1 = t1.center - t1.old_center;
                    Real3 dp2 = t2.center - t2.old_center;
                    Real3 dp  = dp1 - dp2;
                    Real3 dpt = dp - (glm::dot(dp, info.axis) * info.axis);

                    Collision collision;
                    collision.obj1    = &objects[o1i];
                    collision.obj2    = &objects[o2i];
                    collision.t1      = t1i;
                    collision.t2      = t2i;
                    collision.info    = info;
                    collision.dp_tang = dpt;

                    collisions.push_back(collision);

                    objects[o1i].vertex_collisions[t1.vs[0]].push_back(coll_idx);
                    objects[o1i].vertex_collisions[t1.vs[1]].push_back(coll_idx);
                    objects[o1i].vertex_collisions[t1.vs[2]].push_back(coll_idx);
                    objects[o1i].vertex_collisions[t1.vs[3]].push_back(coll_idx);

                    objects[o2i].vertex_collisions[t2.vs[0]].push_back(coll_idx);
                    objects[o2i].vertex_collisions[t2.vs[1]].push_back(coll_idx);
                    objects[o2i].vertex_collisions[t2.vs[2]].push_back(coll_idx);
                    objects[o2i].vertex_collisions[t2.vs[3]].push_back(coll_idx);

                    coll_idx++;
                }
            }
        }
    }
}

void XPBD_collision_vertex_ripositioning(
        std::vector<TetraObject> &objects,
        std::vector<Collision>   &collisions) 
{
    for (TetraObject &obj : objects) {
        for (VertexIndex vi = 0; vi < obj.num_vertices(); vi++) {

            CollisionConstraint constraint(coll_compliance, &obj, vi, Real3(0.0), true);
            
            size_t num_collisions = obj.vertex_collisions[vi].size();
            if (num_collisions == 0) { // || vi >= 8) {
                constraint.active = false;
                obj.vertex_collisions_constraints[vi] = constraint;
                continue;
            }

            for (size_t ci = 0; ci < num_collisions; ci++) {
                Index coll_idx  = obj.vertex_collisions[vi][ci];
                Collision &coll = collisions[coll_idx];

                assert(coll.obj1 == &obj || coll.obj2 == &obj);

                Real3 update = (coll.info.axis * coll.info.penetration) * 0.5;

                if (coll.obj1 == &obj) constraint.goal_position -= update;
                else                   constraint.goal_position += update;
            }

            constraint.goal_position /= (Real) num_collisions;
            constraint.penetration    = glm::length(constraint.goal_position);
            constraint.normal         = glm::normalize(constraint.goal_position);
            constraint.goal_position += obj.positions[vi];

            obj.vertex_collisions_constraints[vi] = constraint;
        }
    }
}

struct StatCollector 
{
    int edge_collisions = 0;
    int face_collisions = 0;

    double total_collision_time = 0.0;
    int steps = 0;

    ~StatCollector() 
    {
        std::cout << "\n--- Simulation Statistics ---" << std::endl;
        std::cout << "Edge Collisions: " << edge_collisions << std::endl;
        std::cout << "Face Collisions: " << face_collisions << std::endl;

        std::cout << "Average Collision Detection Time per Step: " 
                  << (steps > 0 ? (total_collision_time / (double)steps) * 1000.0 : 0.0) 
                  << " ms" << std::endl;

        std::cout << "-----------------------------\n" << std::endl;
    }
};

static StatCollector stat_collector;

void XPBD_step(Scene &scene) 
{

    /*
    for (TetraObject &obj : scene.objects) obj.reset_tetras();

    std::vector<Collision> collisions;

    XPBD_collect_collisions(scene.objects, collisions);

    XPBD_collision_vertex_ripositioning(scene.objects, collisions);

    for (TetraObject &obj : scene.objects) 
    {
        for (VertexIndex vi=0; vi<obj.num_vertices(); vi++) 
        {
            if (obj.inv_masses[vi] == 0.0) continue;
            obj.old_positions[vi]  = obj.positions[vi];
            obj.velocities[vi]    += gravity * delta_t;
            obj.positions[vi]      = obj.positions[vi] + obj.velocities[vi] * delta_t;
        }

        for (int vi=0; vi<obj.num_vertices(); vi++)
            obj.vertex_collisions_constraints[vi].reset();

        for (Edge &edge : obj.edges)          edge.reset();
        for (Tetrahedron &tetra : obj.tetras) tetra.reset();
    }
    for (SpringConstraint &constraint : scene.constraints) constraint.reset();

    for (Cloth &cloth : scene.cloths) {
        for (VertexIndex vi=0; vi<cloth.positions.size(); vi++) {
            if (cloth.inv_masses[vi] == 0.0) continue;
            cloth.old_positions[vi]  = cloth.positions[vi];
            cloth.velocities[vi]    += gravity * delta_t;
            cloth.positions[vi]      = cloth.positions[vi] + cloth.velocities[vi] * delta_t;
        }

        for (ClothEdge &edge : cloth.edges) edge.reset();
    }

    for (int it=0; it<iterations_per_step; it++) 
    {

        for (TetraObject &obj : scene.objects) 
        {
            for (Edge &edge : obj.edges)
                scene.solver.solve(edge, delta_t);

            for (Tetrahedron &tetra : obj.tetras)
                scene.solver.solve(tetra, delta_t);
        }

        for (Cloth &cloth : scene.cloths)
            for (ClothEdge &edge : cloth.edges)
                scene.solver.solve(edge, delta_t);

        for (SpringConstraint &constraint : scene.constraints) 
            scene.solver.solve(constraint, delta_t);

        for (TetraObject &obj : scene.objects) 
            for (int vi=0; vi<obj.num_vertices(); vi++) 
                scene.solver.solve(obj.vertex_collisions_constraints[vi], delta_t);
    }

    for (TetraObject &obj : scene.objects) {
        for (VertexIndex vi=0; vi<obj.num_vertices(); vi++) {
            if (obj.inv_masses[vi] == 0.0) continue;
            if (obj.positions[vi].y < ground_y) obj.positions[vi].y = ground_y;
            obj.velocities[vi] = (obj.positions[vi] - obj.old_positions[vi]) / delta_t;
        }
        obj.update_aabb();
    }

    for (Cloth &cloth : scene.cloths) {
        for (VertexIndex vi=0; vi<cloth.positions.size(); vi++) {
            if (cloth.inv_masses[vi] == 0.0) continue;
            if (cloth.positions[vi].y < ground_y) cloth.positions[vi].y = ground_y;
            cloth.velocities[vi] = (cloth.positions[vi] - cloth.old_positions[vi]) / delta_t;
        }
    }

    */

    // Rigid Objects
    std::vector<RigidCollisionConstraint> rigid_collisions;
    rigid_collisions.reserve(scene.rigid_objects.size() * scene.rigid_objects.size() / 2);

    for (int ri1=0; ri1<scene.rigid_objects.size(); ri1++) 
    {
        for (int ri2=ri1+1; ri2<scene.rigid_objects.size(); ri2++) 
        {
            RigidBox &b1 = scene.getRigidObject(ri1);
            RigidBox &b2 = scene.getRigidObject(ri2);

            if (!b1.aabb.intersects(b2.aabb)) continue;

            RigidCollisionInfo info = SAT_box_box(b1, b2);

            if (!info.intersecting)           continue;
            if (b1.is_static && b2.is_static) continue;

            if (info.owner == 0) 
            {
                assert(info.manifold_size == 2);
                
                RigidCollisionConstraint constraint(
                    coll_compliance, 
                    &b1, 
                    &b2, 
                    info.manifold[0], 
                    info.manifold[1], 
                    info.penetration, 
                    info.axis);
                
                rigid_collisions.push_back(constraint);

                stat_collector.edge_collisions++;

                continue;
            }

            stat_collector.face_collisions++;

            for (int pi=0; pi<info.manifold_size; pi++) 
            {

                RigidCollisionConstraint constraint(
                    coll_compliance, 
                    &b1, 
                    &b2, 
                    info.manifold[pi], 
                    info.manifold[pi], 
                    info.penetration, // / (Real) info.manifold_size, 
                    info.axis);

                rigid_collisions.push_back(constraint);
            }
        }
    }

    for (RigidBox &obj : scene.rigid_objects) 
    {
        obj.update(delta_t, gravity);
    }

    for (FixedRigidSpringConstraint &constraint : scene.fixed_rigid_constraints) 
        constraint.reset();

    for (RigidSpringConstraint &constraint : scene.rigid_constraints) 
        constraint.reset();

    for (RigidCollisionConstraint &constraint : rigid_collisions) 
        constraint.reset();

    // constraints

    for (int it=0; it<iterations_per_step; it++) 
    {
        for (FixedRigidSpringConstraint &constraint : scene.fixed_rigid_constraints) 
            scene.solver.solve(constraint, delta_t);

        for (RigidSpringConstraint &constraint : scene.rigid_constraints) 
            scene.solver.solve(constraint, delta_t);

        for (RigidCollisionConstraint &constraint : rigid_collisions) 
            scene.solver.solve(constraint, delta_t);
    }

    for (RigidBox &obj : scene.rigid_objects) 
    {
        obj.update_velocities(delta_t);
    }

    // velocity solve for dynmaic collision
    for (RigidCollisionConstraint &constraint : rigid_collisions) 
    {
        RigidBox *b1 = constraint.b1;
        RigidBox *b2 = constraint.b2;

        Real3 p1 = constraint.p1;
        Real3 p2 = constraint.p2;

        Real3 r1 = world_to_body(p1, b1->position, b1->orientation);
        Real3 r2 = world_to_body(p2, b2->position, b2->orientation);

        Real3 nw = constraint.n;

        Real3 v1 = b1->velocity + glm::cross(b1->angular_velocity, p1 - b1->position);
        Real3 v2 = b2->velocity + glm::cross(b2->angular_velocity, p2 - b2->position);

        Real3 v = v1 - v2;
        Real vn = glm::dot(v, nw);

        Real3 vt = v - (vn * nw);

        Real fn = std::abs(glm::abs(constraint.lambda) / delta_t);

        if (glm::length(vt) < 1e-6) continue;

        Real3 dv = - glm::normalize(vt) * glm::min(mu_dynamic * fn, glm::length(vt));

        Real w1 = b1->generalized_inverse_mass(r1, world_to_body(nw, Real3(0.0), b1->orientation));
        Real w2 = b2->generalized_inverse_mass(r2, world_to_body(nw, Real3(0.0), b2->orientation));

        Real3 pw = dv / (w1 + w2);

        auto applyVelocityCorrection = [&](RigidBox* body, const Real3& pw, const Real3& r, Real sign) 
        {
            Real3 pb     = world_to_body(pw, Real3(0.0), body->orientation);
            Real3 tau    = glm::cross(r, pb);
            Real3 domega = body->inv_inertia_tensor * tau;
            
            domega = quat_to_rotmat(body->orientation) * domega;

            body->velocity         += sign * pw / body->mass;
            body->angular_velocity += sign * domega;
        };

        if (!b1->is_static) applyVelocityCorrection(b1, pw, r1, 1.0);

        if (!b2->is_static) applyVelocityCorrection(b2, pw, r2, -1.0);
    }


}