#pragma once

#include <vector>
#include <chrono>

#include "object.cpp"
#include "types.cpp"

#include "collision.cpp"
#include "constants.cpp"

#include <stdio.h>

uint64_t steps_per_second    = 4*60;
uint64_t iterations_per_step = 10;
Real delta_t;

extern Real coll_compliance;

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

void XPBD_init() {
    delta_t = 1.0 / steps_per_second;
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

void XPBD_step(Scene &scene) 
{

    for (TetraObject &obj : scene.objects) obj.reset_tetras();

    std::vector<Collision> collisions;
    // auto start = std::chrono::high_resolution_clock::now();
    XPBD_collect_collisions(scene.objects, collisions);
    // auto end      = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // std::cout << "collect coll. time: " << duration << " ms" << std::endl;

    XPBD_collision_vertex_ripositioning(scene.objects, collisions);

    for (TetraObject &obj : scene.objects) {
        for (VertexIndex vi=0; vi<obj.num_vertices(); vi++) {
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

    for (int it=0; it<iterations_per_step; it++) {

        for (TetraObject &obj : scene.objects)
            for (Edge &edge : obj.edges)
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
}