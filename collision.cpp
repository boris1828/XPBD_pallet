#pragma once 
#include <vector>
#include <limits>
#include <array>
#include <numeric>

#include <glm/glm.hpp>

#include "object.cpp"

Real NOT_COLLISION_THRESHOLD = 1e-3;

struct CollisionInfo {
    bool    intersecting;
    Real3   axis;
    Real    penetration;
    uint8_t owner;
};

bool same_side(Real3 v1, Real3 v2, Real3 v3, Real3 v4, Real3 p)
{
    Real3 normal = glm::cross(v2 - v1, v3 - v1);
    Real dotV4   = glm::dot(normal, v4 - v1);
    Real dotP    = glm::dot(normal, p - v1);
    return dotV4 * dotP >= 0;
}

bool point_in_tetrahedron(Real3 v1, Real3 v2, Real3 v3, Real3 v4, Real3 p) {
        return 
            same_side(v2, v3, v4, v1, p) &&
            same_side(v1, v2, v3, v4, p) &&
            same_side(v3, v4, v1, v2, p) &&
            same_side(v4, v1, v2, v3, p);     
}

std::array<bool, 4> points_in_tetrahedron(
        const std::array<Real3, 4>& tet1,
        const std::array<Real3, 4>& tet2) {

    std::array<bool, 4> inside = {false, false, false, false};
    for (int i = 0; i < 4; ++i) {
        inside[i] = point_in_tetrahedron(
            tet1[0], tet1[1], tet1[2], tet1[3], 
            tet2[i]);
    }
    return inside;
}


std::array<Real, 2> project_tetrahedron(const std::array<Real3, 4>& ps, const Real3& axis) {
    Real min_proj = std::numeric_limits<Real>::max();
    Real max_proj = std::numeric_limits<Real>::lowest();
    
    for (const Real3& p : ps) {
        Real proj = glm::dot(p, axis);
        min_proj  = std::min(min_proj, proj);
        max_proj  = std::max(max_proj, proj);
    }

    return {min_proj, max_proj};
}

CollisionInfo SAT_tet_tet(
        TetraObject &obj1, Tetrahedron &tetra1, 
        TetraObject &obj2, Tetrahedron &tetra2) {
    
    tetra1.init_normals_edges();
    tetra2.init_normals_edges();

    std::array<Real3, 4> p1 = {
        obj1.positions[tetra1.vs[0]],
        obj1.positions[tetra1.vs[1]],
        obj1.positions[tetra1.vs[2]],
        obj1.positions[tetra1.vs[3]]
    };

    std::array<Real3, 4> p2 = {
        obj2.positions[tetra2.vs[0]],
        obj2.positions[tetra2.vs[1]],
        obj2.positions[tetra2.vs[2]],
        obj2.positions[tetra2.vs[3]]
    };
    
    std::vector<Real3>   axes;
    std::vector<uint8_t> axes_owner;

    for (const auto& n : tetra1.normals) axes.push_back(n);
    axes_owner.insert(axes_owner.end(), 4, 1);
    for (const auto& n : tetra2.normals) axes.push_back(n);
    axes_owner.insert(axes_owner.end(), 4, 2);

    Real3 center_vec = tetra2.center - tetra1.center;

    for (const auto& e1 : tetra1.edges) {
        for (const auto& e2 : tetra2.edges) {
            Real3 axis = glm::cross(e1, e2);

            if (glm::length(axis) < 1e-6) continue;

            axis = glm::normalize(axis);
            if (glm::dot(axis, center_vec) < 0.0) axis = -axis;

            axes.push_back(axis);
            axes_owner.push_back(0);
        }
    }

    Real min_overlap = std::numeric_limits<Real>::max();
    Real3 min_axis(0.0);

    uint8_t coll_owner = 0;
    for (Index ai = 0; ai < axes.size(); ++ai) {
        const Real3& axis = axes[ai];
        auto [min1, max1] = project_tetrahedron(p1, axis);
        auto [min2, max2] = project_tetrahedron(p2, axis);

        Real overlap = std::min(max1, max2) - std::max(min1, min2);

        if (overlap < NOT_COLLISION_THRESHOLD) return {false, Real3(0.0), 0.0, 0}; 
        if (overlap < min_overlap) {
            min_overlap = overlap;
            min_axis    = axis;
            coll_owner  = axes_owner[ai];
            if (axes_owner[ai] == 2) min_axis = -min_axis;
        }
    }

    // std::array<bool, 4> inside1 = points_in_tetrahedron(p1, p2);
    // std::array<bool, 4> inside2 = points_in_tetrahedron(p2, p1);

    // int count1 = std::accumulate(inside1.begin(), inside1.end(), 0);
    // int count2 = std::accumulate(inside2.begin(), inside2.end(), 0);

    // std::cout << "inside: " << count1 << " : " <<  count2 << "\n";  

    return {true, min_axis, min_overlap, coll_owner};
}