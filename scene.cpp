#pragma once

#include "object.cpp"

struct Scene;

struct Solver {

    void solve(Edge &edge, Real delta_t) {
        Real3 x1 = edge.obj->positions[edge.v1];
        Real3 x2 = edge.obj->positions[edge.v2];

        Real w1 = edge.obj->inv_masses[edge.v1];
        Real w2 = edge.obj->inv_masses[edge.v2];
        Real w  = w1 + w2;

        if (w == 0.0) return;

        Real alpha = edge.compliance / delta_t / delta_t;

        Real C   = distance(x1, x2) - edge.rest_length;
        Real3 dC = (x2-x1) / distance(x2, x1);
        
        Real d_lambda  = (-C - alpha*edge.lambda) / (w + alpha);
        edge.lambda += d_lambda;

        edge.obj->positions[edge.v1] -= d_lambda * w1 * dC;
        edge.obj->positions[edge.v2] += d_lambda * w2 * dC;
    }

    void solve(SpringConstraint &constraint, Real delta_t) {
        Real3 x1 = constraint.obj1->positions[constraint.v1];
        Real3 x2 = constraint.obj2->positions[constraint.v2];

        Real w1 = constraint.obj1->inv_masses[constraint.v1];
        Real w2 = constraint.obj2->inv_masses[constraint.v2];
        Real w  = w1 + w2;

        if (w == 0.0) return;

        Real alpha  = constraint.compliance / delta_t / delta_t;
        Real length = distance(x1, x2);

        if (length < 1e-6) return;

        Real C   = length - constraint.rest_length;
        Real3 dC = (x2-x1) / length;

        Real d_lambda      = (-C -alpha*constraint.lambda) / (w + alpha);
        constraint.lambda += d_lambda;

        constraint.obj1->positions[constraint.v1] -= d_lambda * w1 * dC;
        constraint.obj2->positions[constraint.v2] += d_lambda * w2 * dC;
    }

    void solve(CollisionConstraint &constraint, Real delta_t) {

        if (!constraint.active) return;

        Real3 x      = constraint.obj->positions[constraint.v];
        Real3 old_x  = constraint.obj->old_positions[constraint.v];
        Real3 x_goal = constraint.goal_position;

        Real w = constraint.obj->inv_masses[constraint.v];

        if (w == 0.0) return;

        Real alpha = constraint.compliance / delta_t / delta_t;

        Real C   = distance(x, x_goal);
        Real3 dC = (x-x_goal) / distance(x, x_goal);

        // Real C   = glm::dot(x - old_x, constraint.normal) - constraint.penetration;
        // Real3 dC = constraint.normal;

        Real d_lambda      = (-C - alpha*constraint.lambda) / (w + alpha);
        constraint.lambda += d_lambda;

        constraint.obj->positions[constraint.v] += d_lambda * w * dC;
    }
};

struct Scene {

    std::vector<TetraObject>      objects;
    std::vector<SpringConstraint> constraints;
    Solver solver;

    Scene() = default;

    void addObject(TetraObject& obj) { 
        objects.push_back(std::move(obj)); 
    }

    void addObject(TetraObject&& obj) { 
        objects.push_back(std::move(obj)); 
    }

    void addConstraint(SpringConstraint& constraint) { 
        constraints.push_back(std::move(constraint)); 
    }

    void removeAllConstraints() {
        constraints.clear();
    }

    Real3 center() {
        if (objects.empty()) return Real3(0.0);

        AABB scene_aabb = objects[0].aabb;
        for (Index oi = 0; oi < objects.size(); oi++) {
            scene_aabb.min = glm::min(scene_aabb.min, objects[oi].aabb.min);
            scene_aabb.max = glm::max(scene_aabb.max, objects[oi].aabb.max);
        }
        return (scene_aabb.min + scene_aabb.max) * 0.5;
    }

    TetraObject& getObject(size_t index) {
        if (index < objects.size()) 
            return objects[index];

        throw std::out_of_range("Object index out of range");
    }

};