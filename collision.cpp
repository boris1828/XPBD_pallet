#pragma once 
#include <vector>
#include <limits>
#include <array>
#include <numeric>
#include <algorithm> 
#include <utility>  
#include <iostream>

#include <glm/glm.hpp>

#include "object.cpp"
#include "rigid.cpp"

Real NOT_COLLISION_THRESHOLD = 1e-3;
Real EDGE_CROSS_NOT_VALID_THRESHOLD = 0.98;

struct CollisionInfo 
{
    bool    intersecting;
    Real3   axis;
    Real    penetration;
    uint8_t owner;
};

struct RigidCollisionInfo 
{
    bool    intersecting;
    Real3   axis;
    Real    penetration;
    uint8_t owner;
    std::array<Real3, 16> manifold;
    uint8_t manifold_size;
};

void print_collision_info(const CollisionInfo& info) 
{
    std::cout << "CollisionInfo:\n";
    std::cout << "  intersecting: " << (info.intersecting ? "true" : "false") << "\n";
    std::cout << "  axis: (" 
              << info.axis.x << ", " 
              << info.axis.y << ", " 
              << info.axis.z << ")\n";
    std::cout << "  penetration: " << info.penetration << "\n";
    std::cout << "  owner: " << static_cast<int>(info.owner) << "\n";
}

void print_collision_info(const RigidCollisionInfo& info) 
{
    std::cout << "CollisionInfo:\n";
    std::cout << "  intersecting: " << (info.intersecting ? "true" : "false") << "\n";
    std::cout << "  axis: (" 
              << info.axis.x << ", " 
              << info.axis.y << ", " 
              << info.axis.z << ")\n";
    std::cout << "  penetration: " << info.penetration << "\n";
    std::cout << "  owner: " << static_cast<int>(info.owner) << "\n";
    std::cout << "  manifold size: " << static_cast<int>(info.manifold_size) << "\n";
    std::cout << "  manifold points:\n";
    for (int i = 0; i < info.manifold_size; ++i)
        std::cout << "    (" 
                  << info.manifold[i].x << ", " 
                  << info.manifold[i].y << ", " 
                  << info.manifold[i].z << ")\n";
}


bool same_side(Real3 v1, Real3 v2, Real3 v3, Real3 v4, Real3 p)
{
    Real3 normal = glm::cross(v2 - v1, v3 - v1);
    Real dotV4   = glm::dot(normal, v4 - v1);
    Real dotP    = glm::dot(normal, p - v1);
    return dotV4 * dotP >= 0;
}

bool point_in_tetrahedron(Real3 v1, Real3 v2, Real3 v3, Real3 v4, Real3 p) 
{
        return 
            same_side(v2, v3, v4, v1, p) &&
            same_side(v1, v2, v3, v4, p) &&
            same_side(v3, v4, v1, v2, p) &&
            same_side(v4, v1, v2, v3, p);     
}

std::array<bool, 4> points_in_tetrahedron(const std::array<Real3, 4>& tet1, const std::array<Real3, 4>& tet2) 
{
    std::array<bool, 4> inside = {false, false, false, false};
    for (int i = 0; i < 4; ++i) {
        inside[i] = point_in_tetrahedron(
            tet1[0], tet1[1], tet1[2], tet1[3], 
            tet2[i]);
    }
    return inside;
}

std::pair<Real, Real> project_box(const RigidBox& box, const Real3& axis) 
{
    Real min_proj = glm::dot(box.world_vertices[0], axis);
    Real max_proj = min_proj;

    for (size_t i = 1; i < box.world_vertices.size(); ++i) 
    {
        Real proj = glm::dot(box.world_vertices[i], axis);
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }

    return {min_proj, max_proj};
}

std::array<Real, 2> project_tetrahedron(const std::array<Real3, 4>& ps, const Real3& axis) 
{
    Real min_proj = std::numeric_limits<Real>::max();
    Real max_proj = std::numeric_limits<Real>::lowest();
    
    for (const Real3& p : ps) {
        Real proj = glm::dot(p, axis);
        min_proj  = std::min(min_proj, proj);
        max_proj  = std::max(max_proj, proj);
    }

    return {min_proj, max_proj};
}

CollisionInfo SAT_tet_tet(TetraObject &obj1, Tetrahedron &tetra1, TetraObject &obj2, Tetrahedron &tetra2) 
{    
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

    return {true, min_axis, min_overlap, coll_owner};
}

RigidCollisionInfo SAT_box_box(RigidBox &b1, RigidBox &b2) 
{
    std::array<Real3, 15>   axes;
    std::array<uint8_t, 15> axes_owner;
    int axis_count = 0;

    std::array<Real3, 3> b1_normals = quat_to_axes(b1.orientation);
    std::array<Real3, 3> b2_normals = quat_to_axes(b2.orientation);

    for (const auto& n : b1_normals) 
    {
        axes_owner[axis_count] = 1;
        axes[axis_count++] = n;
    }

    for (const auto& n : b2_normals) 
    {
        axes_owner[axis_count] = 2;
        axes[axis_count++] = n;
    }

    auto get_edges_directions = [](const RigidBox& b) -> std::array<Real3, 3> 
    {
        return 
        {
            b.world_vertices[1] - b.world_vertices[0],
            b.world_vertices[3] - b.world_vertices[0],
            b.world_vertices[4] - b.world_vertices[0]
        };
    };

    std::array<Real3, 3> b1_edges = get_edges_directions(b1);
    std::array<Real3, 3> b2_edges = get_edges_directions(b2);

    Real3 center_vec = b2.position - b1.position;

    for (const auto& e1 : b1_edges) 
    {
        for (const auto& e2 : b2_edges) 
        {
            Real3 axis = glm::cross(e1, e2);

            if (glm::length(axis) < 1e-6) continue;

            axis = glm::normalize(axis);
            if (glm::dot(axis, center_vec) < 0.0) axis = -axis;

            bool to_similar = false;
            for (const auto& a : axes) 
            {
                if (std::abs(glm::dot(a, axis)) > EDGE_CROSS_NOT_VALID_THRESHOLD) 
                {
                    to_similar = true;
                    break;
                }
            }

            axes[axis_count] = axis;

            if (to_similar) axes_owner[axis_count] = 3;
            else            axes_owner[axis_count] = 0;

            axis_count++;
        }
    }

    Real min_overlap = std::numeric_limits<Real>::max();
    Real3 min_axis(0.0);

    uint8_t coll_owner = 0;
    for (Index ai = 0; ai < axis_count; ++ai) 
    {
        Real3 axis = axes[ai];
        auto [min1, max1] = project_box(b1, axis);
        auto [min2, max2] = project_box(b2, axis);

        Real overlap = std::min(max1, max2) - std::max(min1, min2);

        if (overlap < NOT_COLLISION_THRESHOLD) return {false, Real3(0.0), 0.0, 0}; 
        if (overlap < min_overlap && axes_owner[ai] != 3) 
        {
            if (glm::dot(axis, center_vec) > 0.0) axis = -axis;
            min_overlap = overlap;
            min_axis    = axis;
            coll_owner  = axes_owner[ai];
        }
    }

    if (coll_owner == 0) 
    {    
        auto get_closests_points_on_segments = [](Real3 p1, Real3 p2, Real3 q1, Real3 q2) -> std::pair<Real3, Real3> 
        {
            Real3 u  = p2 - p1;
            Real3 v  = q2 - q1;
            Real3 w0 = p1 - q1;

            Real a   = glm::dot(u,u);  // |u|^2
            Real b   = glm::dot(u,v);
            Real c   = glm::dot(v,v);  // |v|^2
            Real d   = glm::dot(u,w0);
            Real e   = glm::dot(v,w0);
            Real den = a*c - b*b;

            if (den > 1e-6) // not parallel
            {  

                Real s = (b*e - c*d) / den;
                Real t = (a*e - b*d) / den;

                s = glm::clamp(s, 0.0, 1.0);
                t = glm::clamp(t, 0.0, 1.0);

                Real3 cp1 = p1 + s * u;
                Real3 cp2 = q1 + t * v;

                return {cp1, cp2};
            } 

            // parallel 
            Real3 mid1 = 0.5 * (p1 + p2);

            Real t = glm::dot(mid1 - q1, v) / glm::dot(v, v);
            
            t = glm::clamp(t, 0.0, 1.0);

            Real3 cp1 = mid1;
            Real3 cp2 = q1 + t * v;

            return {cp1, cp2};
        };

        auto get_edges_segments = [](const RigidBox& b) -> std::array<std::pair<Real3, Real3>, 12> 
        {
            return std::array<std::pair<Real3, Real3>, 12> 
            {
                std::make_pair(b.world_vertices[0], b.world_vertices[1]),
                std::make_pair(b.world_vertices[0], b.world_vertices[3]),
                std::make_pair(b.world_vertices[0], b.world_vertices[4]),
                std::make_pair(b.world_vertices[2], b.world_vertices[1]),
                std::make_pair(b.world_vertices[2], b.world_vertices[3]),
                std::make_pair(b.world_vertices[2], b.world_vertices[6]),
                std::make_pair(b.world_vertices[5], b.world_vertices[4]),
                std::make_pair(b.world_vertices[5], b.world_vertices[6]),
                std::make_pair(b.world_vertices[5], b.world_vertices[1]),
                std::make_pair(b.world_vertices[7], b.world_vertices[4]),
                std::make_pair(b.world_vertices[7], b.world_vertices[6]),
                std::make_pair(b.world_vertices[7], b.world_vertices[3])
            };
        };

        std::array<std::pair<Real3, Real3>, 12> b1_edges_segs = get_edges_segments(b1);
        std::array<std::pair<Real3, Real3>, 12> b2_edges_segs = get_edges_segments(b2);

        Real min_dist = std::numeric_limits<Real>::max();

        std::array<Real3, 16> manifold;

        for (const auto& e1 : b1_edges_segs) 
        {
            for (const auto& e2 : b2_edges_segs) 
            {
                auto [cp1, cp2] = get_closests_points_on_segments(e1.first, e1.second, e2.first, e2.second);
                Real dist = glm::length(cp1 - cp2);
                if (dist < min_dist) 
                {
                    min_dist    = dist;
                    manifold[0] = cp1;
                    manifold[1] = cp2;
                }
            }
        }

        // return {true, min_axis, min_overlap, coll_owner, manifold, 2};
        return {true, min_axis, min_dist, coll_owner, manifold, 2}; // TOCHECK: is min_dist circ= min_overlap
    }

    RigidBox *ref_box   = &b1;
    RigidBox *inc_box   = &b2;
    Real3     coll_axis = min_axis;

    if (coll_owner == 2) 
    {
        ref_box   = &b2;
        inc_box   = &b1;
        coll_axis = -min_axis;
    }

    auto get_projected_points = [](const RigidBox *box, Real3 axis) -> std::array<Real, 8> 
    {
        std::array<Real, 8> projected_points;
        for (int vi=0; vi<8; vi++) 
            projected_points[vi] = glm::dot(box->world_vertices[vi], axis);
        return projected_points;
    };

    auto get_min_max = [](const std::array<Real, 8> &points) -> std::pair<VertexIndex, VertexIndex>
    {
        VertexIndex min_idx = 0;
        VertexIndex max_idx = 0;

        for (VertexIndex i = 1; i < 8; i++) 
        {
            if (points[i] < points[min_idx]) min_idx = i;
            if (points[i] > points[max_idx]) max_idx = i;
        }

        return {min_idx, max_idx};
    };

    std::array<Real, 8> ref_projected_points = get_projected_points(ref_box, coll_axis);
    std::array<Real, 8> inc_projected_points = get_projected_points(inc_box, coll_axis);

    auto [ref_min, ref_max] = get_min_max(ref_projected_points);
    auto [inc_min, inc_max] = get_min_max(inc_projected_points);

    VertexIndex ref_support_vertex = ref_min;
    VertexIndex inc_support_vertex = inc_max;

    if (ref_projected_points[ref_min] < inc_projected_points[inc_min]) 
    {
        ref_support_vertex = ref_max;
        inc_support_vertex = inc_min;
    }

    static constexpr VertexIndex FORW[8] = {1, 2, 3, 0, 5, 6, 7, 4};
    static constexpr VertexIndex BACK[8] = {3, 0, 1, 2, 7, 4, 5, 6};
    static constexpr VertexIndex UP[8]   = {4, 5, 6, 7, 0, 0, 0, 0};
    static constexpr VertexIndex DOWN[8] = {0, 0, 0, 0, 0, 1, 2, 3};

    auto get_faces_with_point = [](VertexIndex v) -> std::array<std::array<VertexIndex, 4>, 3>
    {
        std::array<std::array<VertexIndex, 4>, 3> faces;

        if (v < 4) 
        {
            faces[0] = {0, 1, 2, 3};
            faces[1] = {v, FORW[v], UP[FORW[v]], BACK[UP[FORW[v]]]};
            faces[2] = {v, BACK[v], UP[BACK[v]], FORW[UP[BACK[v]]]};
        } 
        else 
        {
            faces[0] = {4, 5, 6, 7};
            faces[1] = {v, FORW[v], DOWN[FORW[v]], BACK[DOWN[FORW[v]]]};
            faces[2] = {v, BACK[v], DOWN[BACK[v]], FORW[DOWN[BACK[v]]]};
        }

        return faces;
    };

    auto ref_faces = get_faces_with_point(ref_support_vertex);
    auto inc_faces = get_faces_with_point(inc_support_vertex);

    auto compute_face_normal = [](const RigidBox* box, const std::array<VertexIndex, 4> &face) -> Real3 
    {
        Real3 v0 = box->world_vertices[face[0]];
        Real3 v1 = box->world_vertices[face[1]];
        Real3 v2 = box->world_vertices[face[3]];
        Real3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        Real3 face_center = (box->world_vertices[face[0]] +
                             box->world_vertices[face[1]] +
                             box->world_vertices[face[2]] +
                             box->world_vertices[face[3]]) * 0.25;

        if (glm::dot(normal, box->position - face_center) > 0.0f) 
        {
            normal = -normal;
        }

        return normal;
    };

    Index ref_face_idx;
    Index inc_face_idx;
    Real most_parallel;
    Real most_anti_parallel;

    most_parallel     = -1.0; 
    most_anti_parallel = 1.0; 

    for (Index i = 0; i < 3; i++)
    {
        Real3 ref_normal = compute_face_normal(ref_box, ref_faces[i]);
        Real3 inc_normal = compute_face_normal(inc_box, inc_faces[i]);
        Real ref_dot     = glm::dot(ref_normal, - coll_axis); 
        Real inc_dot     = glm::dot(inc_normal, - coll_axis);  

        if (ref_dot > most_parallel) 
        {
            most_parallel = ref_dot;
            ref_face_idx = i;
        }

        if (inc_dot < most_anti_parallel) 
        {
            most_anti_parallel = inc_dot;
            inc_face_idx = i;
        }
    }

    // Sutherland-Hodgman

    Real3 side_planes[5][2];

    Real3 ref_face_center = (ref_box->world_vertices[ref_faces[ref_face_idx][0]] +
                             ref_box->world_vertices[ref_faces[ref_face_idx][1]] +
                             ref_box->world_vertices[ref_faces[ref_face_idx][2]] +
                             ref_box->world_vertices[ref_faces[ref_face_idx][3]]) * 0.25;

    side_planes[0][0] = ref_face_center;
    side_planes[0][1] = - coll_axis;

    for (int i=1; i<5; i++) 
    {
        Real3 v1 = ref_box->world_vertices[ref_faces[ref_face_idx][i-1]];
        Real3 v2 = ref_box->world_vertices[ref_faces[ref_face_idx][(i)%4]];
        Real3 edge_center = (v1 + v2) * 0.5;
        Real3 side_plane_normal = glm::normalize(glm::cross((v2-v1), - coll_axis));

        if (glm::dot(side_plane_normal, (edge_center - ref_face_center)) < 0.0)
            side_plane_normal = -side_plane_normal;

        side_planes[i][0] = edge_center;
        side_planes[i][1] = side_plane_normal;
    }

    auto signed_distance = [](const Real3& P0, const Real3& n, const Real3& q) 
    {
        return glm::dot(q - P0, n);
    };

    auto intersect_segment_plane = [&](const Real3& A, const Real3& B, const Real3& P0, const Real3& n) -> Real3 
    {
        Real dA = signed_distance(P0, n, A); 
        Real dB = signed_distance(P0, n, B); 
        Real denom = dA - dB; 

        Real t = dA / denom; 
        Real3 I = A + (B - A) * t;
        return I;
    };

    std::array<Real3, 16> manifold;
    uint8_t manifold_size = 4;
    for (int i=0; i<4; i++)
        manifold[i] = inc_box->world_vertices[inc_faces[inc_face_idx][i]];

    for (int pi=0; pi<5; pi++) 
    {
        std::array<Real3, 16> new_manifold;
        uint8_t new_manifold_size = 0;

        Real3 plane_center = side_planes[pi][0];
        Real3 plane_normal = side_planes[pi][1];

        for (int mi=0; mi<manifold_size; mi++) {

            Real3 v1 = manifold[mi];
            Real3 v2 = manifold[(mi+1)%manifold_size];

            bool v1_inside = signed_distance(plane_center, plane_normal, v1) <= 0.0;
            bool v2_inside = signed_distance(plane_center, plane_normal, v2) <= 0.0;

            if (v1_inside && v2_inside) 
            {
                new_manifold[new_manifold_size] = v1;
                new_manifold_size++;
            } 
            else if (v1_inside && !v2_inside) 
            {
                Real3 inters = intersect_segment_plane(v1, v2, plane_center, plane_normal);
                new_manifold[new_manifold_size] = v1;
                new_manifold_size++;
                new_manifold[new_manifold_size] = inters;
                new_manifold_size++;
            } 
            else if (!v1_inside && v2_inside) 
            {
                Real3 inters = intersect_segment_plane(v1, v2, plane_center, plane_normal);
                new_manifold[new_manifold_size] = inters;
                new_manifold_size++;
            }
        }
        manifold_size = new_manifold_size;
        for (int mi=0; mi<manifold_size; mi++)
            manifold[mi] = new_manifold[mi];
    }

    return {true, min_axis, min_overlap, coll_owner, manifold, manifold_size};
}