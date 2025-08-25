#pragma once

#include <cstdint>
#include <vector>
#include <set>

#include "types.cpp"
#include "mesh.cpp"
#include "AABB.cpp"
#include "constants.cpp"

extern Real box_edge_compliance;

struct TetraObject {

    TetraMesh mesh;
    std::vector<Tetrahedron> tetras;

    std::vector<Edge>        edges;
    std::vector<Real3>       positions;
    std::vector<Real3>       old_positions;
    std::vector<Real3>       velocities;
    std::vector<Real>        inv_masses;
    std::vector<std::vector<uint32_t>> vertex_edges;
    std::vector<std::vector<uint32_t>> vertex_tetras;

    std::vector<std::vector<uint32_t>> vertex_collisions;
    std::vector<CollisionConstraint>   vertex_collisions_constraints;

    AABB aabb;

    TetraObject(TetraObject&& other) noexcept
        : mesh(std::move(other.mesh)),
          tetras(std::move(other.tetras)),
          edges(std::move(other.edges)),
          positions(std::move(other.positions)),
          old_positions(std::move(other.old_positions)),
          velocities(std::move(other.velocities)),
          inv_masses(std::move(other.inv_masses)),
          vertex_edges(std::move(other.vertex_edges)),
          vertex_tetras(std::move(other.vertex_tetras)),
          vertex_collisions(std::move(other.vertex_collisions)),
          vertex_collisions_constraints(std::move(other.vertex_collisions_constraints)),
          aabb(other.aabb)
    {
        mesh.vertices = &positions;
        for (auto& e : edges)  e.setObject(this);
        for (auto& t : tetras) t.setObject(this);
    }

    TetraObject& operator=(TetraObject&& other) noexcept {
        if (this != &other) {
            mesh              = std::move(other.mesh);
            tetras            = std::move(other.tetras);
            edges             = std::move(other.edges);
            positions         = std::move(other.positions);
            old_positions     = std::move(other.old_positions);
            velocities        = std::move(other.velocities);
            inv_masses        = std::move(other.inv_masses);
            vertex_edges      = std::move(other.vertex_edges);
            vertex_tetras     = std::move(other.vertex_tetras);
            vertex_collisions = std::move(other.vertex_collisions);
            vertex_collisions_constraints = std::move(other.vertex_collisions_constraints);
            aabb              = other.aabb;

            mesh.vertices = &positions;
            for (auto& e : edges)  e.setObject(this);
            for (auto& t : tetras) t.setObject(this);
        }
        return *this;
    }

    TetraObject(std::vector<Real3> vs) : 
            positions(vs),
            tetras(),
            edges(),
            old_positions(vs.size(), Real3(0.0)),
            velocities(vs.size(), Real3(0.0)),
            inv_masses(vs.size(), 1.0),
            vertex_edges(vs.size()),
            vertex_tetras(vs.size()),
            vertex_collisions(vs.size()),
            vertex_collisions_constraints(vs.size())      
    {}

    void init_tetras_and_edges(std::vector<Tetrahedron> trs) {
        tetras = std::move(trs);
        edges.clear();

        // init tetras
        TetraIndex tetra_idx = 0;
        for (Tetrahedron &t : tetras) {
            vertex_tetras[t.vs[0]].push_back(tetra_idx);
            vertex_tetras[t.vs[1]].push_back(tetra_idx);
            vertex_tetras[t.vs[2]].push_back(tetra_idx);
            vertex_tetras[t.vs[3]].push_back(tetra_idx);
            t.setObject(this);
            tetra_idx++;
        }

        // init edges
        std::set<std::pair<VertexIndex, VertexIndex>> uniqueEdges;
        for (auto& t : tetras) {
            for (int i = 0; i < 4; ++i)
                for (int j = i + 1; j < 4; ++j)
                    uniqueEdges.insert({
                                    std::min(t.vs[i], t.vs[j]), 
                                    std::max(t.vs[i], t.vs[j])});
        }

        EdgeIndex edge_idx = 0;
        for (auto& e : uniqueEdges) {
            VertexIndex v1 = e.first;
            VertexIndex v2 = e.second;
            Real length    = distance(positions[v1], positions[v2]);
            Edge edge(box_edge_compliance, this, v1, v2, length);

            edges.push_back(edge);

            vertex_edges[v1].push_back(edge_idx);
            vertex_edges[v2].push_back(edge_idx);
            edge_idx++;
        }
        
        // create tetra mesh object
        mesh = TetraMesh(&positions, tetras, edges);

        update_aabb();

    }

    void translate(Real3 translation) {
        for (auto& pos : positions) {
            pos += translation;
        }
        update_aabb();
    }

    void update_aabb() {
        if (positions.empty()) return;

        Real3 min = positions[0];
        Real3 max = positions[0];

        for (const auto& pos : positions) {
            min = glm::min(min, pos);
            max = glm::max(max, pos);
        }

        aabb = AABB(min, max);

        for (auto& t : tetras) t.update_bounding_sphere();

    }

    void draw() {
        mesh.drawWireframe();
        // mesh.drawFaceNormals(tetras);
        // mesh.drawPoints();
    }

    uint32_t num_vertices() const {
        return positions.size();
    }

    uint32_t num_edges() const {
        return edges.size();
    }

    uint32_t num_tetras() const {
        return tetras.size();
    }

    void set_velocity(Real3 velocity) {
        std::fill(velocities.begin(), velocities.end(), velocity);
    }

};

std::array<Real3, 4> get_tetra_points(TetraObject* obj, VertexIndex vs[4]) {
    return {obj->positions[vs[0]],
            obj->positions[vs[1]],
            obj->positions[vs[2]],
            obj->positions[vs[3]]};
}

TetraObject create_box(Real3 starting_corner, Real width, Real height, Real depth) {

    std::vector<Real3> ps;
    std::vector<Tetrahedron> tetras;

    Real hheight = height / 2.0;
    Real hwidth  = width  / 2.0;
    Real hdepth  = depth  / 2.0;

    Real3 C(starting_corner.x + hwidth, 
            starting_corner.y + hheight, 
            starting_corner.z + hdepth);

    Real3 A1(C.x - hwidth, C.y + hheight, C.z - hdepth);
    Real3 A2(C.x - hwidth, C.y + hheight, C.z + hdepth);
    Real3 A3(C.x + hwidth, C.y + hheight, C.z + hdepth);
    Real3 A4(C.x + hwidth, C.y + hheight, C.z - hdepth);
    Real3 B1(C.x - hwidth, C.y - hheight, C.z - hdepth);
    Real3 B2(C.x - hwidth, C.y - hheight, C.z + hdepth);
    Real3 B3(C.x + hwidth, C.y - hheight, C.z + hdepth);
    Real3 B4(C.x + hwidth, C.y - hheight, C.z - hdepth);

    ps.push_back(A1); // 0
    ps.push_back(A2); // 1
    ps.push_back(A3); // 2
    ps.push_back(A4); // 3
    ps.push_back(B1); // 4
    ps.push_back(B2); // 5
    ps.push_back(B3); // 6
    ps.push_back(B4); // 7
    ps.push_back(C);  // 8

    TetraObject obj(ps);
    Real volume;

    volume = tetra_volume(ps[1], ps[0], ps[2], ps[8]);
    tetras.push_back({0.0, nullptr, 1, 0, 2, 8, volume});
    volume = tetra_volume(ps[2], ps[0], ps[3], ps[8]);
    tetras.push_back({0.0, nullptr, 2, 0, 3, 8, volume});
    volume = tetra_volume(ps[4], ps[5], ps[7], ps[8]);
    tetras.push_back({0.0, nullptr, 4, 5, 7, 8, volume});
    volume = tetra_volume(ps[7], ps[5], ps[6], ps[8]);
    tetras.push_back({0.0, nullptr, 7, 5, 6, 8, volume});
    volume = tetra_volume(ps[7], ps[6], ps[3], ps[8]);
    tetras.push_back({0.0, nullptr, 7, 6, 3, 8, volume});
    volume = tetra_volume(ps[3], ps[6], ps[2], ps[8]);
    tetras.push_back({0.0, nullptr, 3, 6, 2, 8, volume});
    volume = tetra_volume(ps[1], ps[4], ps[0], ps[8]);
    tetras.push_back({0.0, nullptr, 1, 4, 0, 8, volume});
    volume = tetra_volume(ps[5], ps[4], ps[1], ps[8]);
    tetras.push_back({0.0, nullptr, 5, 4, 1, 8, volume});
    volume = tetra_volume(ps[3], ps[4], ps[7], ps[8]);
    tetras.push_back({0.0, nullptr, 3, 4, 7, 8, volume});
    volume = tetra_volume(ps[0], ps[4], ps[3], ps[8]);
    tetras.push_back({0.0, nullptr, 0, 4, 3, 8, volume});
    volume = tetra_volume(ps[1], ps[6], ps[5], ps[8]);
    tetras.push_back({0.0, nullptr, 1, 6, 5, 8, volume});
    volume = tetra_volume(ps[2], ps[6], ps[1], ps[8]);
    tetras.push_back({0.0, nullptr, 2, 6, 1, 8, volume});

    obj.init_tetras_and_edges(tetras);

    return obj;
}

/*

TetraObject create_cube(Real3 center, float size) {

    std::vector<Real3> positions;
    std::vector<Tetrahedron> tetras;

    float hsize = size / 2.0;

    Real3 C(center);
    Real3 A1(C.x - hsize, C.y + hsize, C.z - hsize);
    Real3 A2(C.x - hsize, C.y + hsize, C.z + hsize);
    Real3 A3(C.x + hsize, C.y + hsize, C.z + hsize);
    Real3 A4(C.x + hsize, C.y + hsize, C.z - hsize);
    Real3 B1(C.x - hsize, C.y - hsize, C.z - hsize);
    Real3 B2(C.x - hsize, C.y - hsize, C.z + hsize);
    Real3 B3(C.x + hsize, C.y - hsize, C.z + hsize);
    Real3 B4(C.x + hsize, C.y - hsize, C.z - hsize);

    positions.push_back(A1); // 0
    positions.push_back(A2); // 1
    positions.push_back(A3); // 2
    positions.push_back(A4); // 3
    positions.push_back(B1); // 4
    positions.push_back(B2); // 5
    positions.push_back(B3); // 6
    positions.push_back(B4); // 7
    positions.push_back(C);  // 8

    tetras.push_back({1, 0, 2, 8});
    tetras.push_back({2, 0, 3, 8});
    tetras.push_back({4, 5, 7, 8});
    tetras.push_back({7, 5, 6, 8});
    tetras.push_back({7, 6, 3, 8});
    tetras.push_back({3, 6, 2, 8});
    tetras.push_back({1, 4, 0, 8});
    tetras.push_back({5, 4, 1, 8});
    tetras.push_back({3, 4, 7, 8});
    tetras.push_back({0, 4, 3, 8});
    tetras.push_back({1, 6, 5, 8});
    tetras.push_back({2, 6, 1, 8});

    return TetraObject(positions, tetras);
}

TetraObject create_tetrahedron(Real3 center, float size) {
    std::vector<Real3> positions;
    std::vector<Tetrahedron> tetras;

    float h = size / 2.0;

    Real3 p1 = center + Real3(-h, -h, -h);
    Real3 p2 = center + Real3( h, -h, -h);
    Real3 p3 = center + Real3( 0,  h, -h);
    Real3 p4 = center + Real3( 0,  0,  h);

    positions.push_back(p1); // 0
    positions.push_back(p2); // 1
    positions.push_back(p3); // 2
    positions.push_back(p4); // 3

    tetras.push_back({0, 1, 2, 3});

    return TetraObject(positions, tetras);
}

TetraObject create_tetrahedron(Real3 p1, Real3 p2, Real3 p3, Real3 p4) {
    std::vector<Real3> positions;
    std::vector<Tetrahedron> tetras;

    positions.push_back(p1);
    positions.push_back(p2);
    positions.push_back(p3);
    positions.push_back(p4);

    tetras.push_back({0, 1, 2, 3});

    return TetraObject(positions, tetras);
}

uint32_t index(uint32_t i, uint32_t j, uint32_t k, uint32_t width, uint32_t depth) {
    return i * width * depth + j * depth + k;
}

void add_tetras(
    std::vector<Tetrahedron> &tetras,
    uint32_t v0, uint32_t v1, uint32_t v2, 
    uint32_t v3, uint32_t v4, uint32_t v5, 
    uint32_t v6, uint32_t v7, uint32_t c) {

    tetras.push_back({v1, v0, v2, c});
    tetras.push_back({v2, v0, v3, c});
    tetras.push_back({v4, v5, v7, c});
    tetras.push_back({v7, v5, v6, c});
    tetras.push_back({v7, v6, v3, c});
    tetras.push_back({v3, v6, v2, c});
    tetras.push_back({v1, v4, v0, c});
    tetras.push_back({v5, v4, v1, c});
    tetras.push_back({v3, v4, v7, c});
    tetras.push_back({v0, v4, v3, c});
    tetras.push_back({v1, v6, v5, c});
    tetras.push_back({v2, v6, v1, c});

}

TetraObject create_stacked_cubes(Real3 center, uint32_t height, uint32_t width, uint32_t depth, Real size)
{
    std::vector<Real3> points((height+1) * (width+1) * (depth+1), Real3(0.0));
    std::vector<Real3> centers(height * width * depth, Real3(0.0));

    for (uint32_t i = 0; i <= height; ++i) {
        for (uint32_t j = 0; j <= width; ++j) {
            for (uint32_t k = 0; k <= depth; ++k) {
                Real3 p(center.x + j * size, center.y + i * size, center.z + k * size);
                points[index(i, j, k, width+1, depth+1)] = p;
            }
        }
    }

    for (uint32_t i = 0; i < height; ++i) {
        for (uint32_t j = 0; j < width; ++j) {
            for (uint32_t k = 0; k < depth; ++k) {
                Real3 c(center.x + j * size + size/2.0, 
                       center.y + i * size + size/2.0, 
                       center.z + k * size + size/2.0);
                centers[index(i, j, k, width, depth)] = c;
            }
        }
    }

    std::vector<Real3> positions(points.size() + centers.size());
    std::copy(points.begin(),  points.end(),  positions.begin());
    std::copy(centers.begin(), centers.end(), positions.begin() + points.size());

    std::vector<Tetrahedron> tetras;

    for (uint32_t i = 0; i < height; ++i) {
        for (uint32_t j = 0; j < width; ++j) {
            for (uint32_t k = 0; k < depth; ++k) {

                uint32_t Cidx  = index(i, j, k, width, depth) + points.size();
                uint32_t A1idx = index(i+1, j,   k,   width+1, depth+1);
                uint32_t A2idx = index(i+1, j,   k+1, width+1, depth+1);
                uint32_t A3idx = index(i+1, j+1, k+1, width+1, depth+1);
                uint32_t A4idx = index(i+1, j+1, k,   width+1, depth+1);
                uint32_t B1idx = index(i,   j,   k,   width+1, depth+1);
                uint32_t B2idx = index(i,   j,   k+1, width+1, depth+1);
                uint32_t B3idx = index(i,   j+1, k+1, width+1, depth+1);
                uint32_t B4idx = index(i,   j+1, k,   width+1, depth+1);

                add_tetras(
                    tetras, 
                    A1idx, A2idx, A3idx, A4idx, 
                    B1idx, B2idx, B3idx, B4idx, 
                    Cidx);
            }
        }
    }

    return TetraObject(positions, tetras);
}

*/