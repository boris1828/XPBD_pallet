#pragma once

#include <cstdint>
#include <vector>
#include <set>
#include <array>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

#include "types.h"
#include "mesh.cpp"
#include "AABB.cpp"
#include "settings.cpp"

enum class Border {
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
};

struct Cloth {
    std::vector<Real3> positions;
    std::vector<Real3> old_positions;
    std::vector<Real3> velocities;
    std::vector<Real> inv_masses;
    std::vector<ClothEdge> edges;
    
    // Struttura per tenere traccia della griglia
    uint32_t grid_width;
    uint32_t grid_height;
    Real cell_size;
    
    ClothMesh mesh;
    AABB aabb;
    
    Cloth(Cloth&& other) noexcept
        : positions(std::move(other.positions)),
          old_positions(std::move(other.old_positions)),
          velocities(std::move(other.velocities)),
          inv_masses(std::move(other.inv_masses)),
          edges(std::move(other.edges)),
          grid_width(other.grid_width),
          grid_height(other.grid_height),
          cell_size(other.cell_size),
          mesh(std::move(other.mesh)),
          aabb(other.aabb)
    {
        mesh.vertices = &positions;
        for (auto& e : edges) e.setObject(this);
    }
    
    Cloth& operator=(Cloth&& other) noexcept {
        if (this != &other) {
            positions = std::move(other.positions);
            old_positions = std::move(other.old_positions);
            velocities = std::move(other.velocities);
            inv_masses = std::move(other.inv_masses);
            edges = std::move(other.edges);
            grid_width = other.grid_width;
            grid_height = other.grid_height;
            cell_size = other.cell_size;
            mesh = std::move(other.mesh);
            aabb = other.aabb;
            
            mesh.vertices = &positions;
            for (auto& e : edges) e.setObject(this);
        }
        return *this;
    }
    
    Cloth(Real3 top_left, Real3 bottom_right, uint32_t density, Real total_mass, Real edge_compliance = 0.0)
        : grid_width(density), grid_height(density), cell_size(0.0)
    {
        
        Real3 size = bottom_right - top_left;
        
        // Calcola cell_size separatamente per ogni dimensione
        Real cell_size_x = size.x / (density - 1);
        Real cell_size_y = size.y / (density - 1);
        Real cell_size_z = size.z / (density - 1);
        cell_size = glm::max(cell_size_x, glm::max(cell_size_y, cell_size_z));
        
        
        positions.reserve(density * density);
        old_positions.reserve(density * density);
        velocities.reserve(density * density);
        inv_masses.reserve(density * density);
        
        Real mass_per_particle = total_mass / (density * density);
        Real inv_mass = mass_per_particle > 0.0 ? 1.0 / mass_per_particle : 0.0;
        
        
        for (uint32_t j = 0; j < density; ++j) {
            for (uint32_t i = 0; i < density; ++i) {
                Real u = static_cast<Real>(i) / (density - 1);
                Real v = static_cast<Real>(j) / (density - 1);
                
                // Usa tutte e tre le dimensioni per posizionare i punti
                Real3 position = top_left + Real3(u * size.x, 0.0, v * size.z);
                positions.push_back(position);
                old_positions.push_back(position);
                velocities.push_back(Real3(0.0));
                inv_masses.push_back(inv_mass);
            }
        }
        
        // Crea gli edge constraints
        create_edge_constraints(edge_compliance);
        
        // Crea la mesh per il rendering
        create_mesh();
        
        update_aabb();
    }
        
private:
    void create_edge_constraints(Real compliance) {
        // Crea edges orizzontali e verticali (strutturali)
        for (uint32_t j = 0; j < grid_height; ++j) {
            for (uint32_t i = 0; i < grid_width; ++i) {
                uint32_t idx = j * grid_width + i;
                
                // Edge orizzontale (destra)
                if (i < grid_width - 1) {
                    uint32_t right_idx = j * grid_width + (i + 1);
                    add_edge(compliance, idx, right_idx);
                }
                
                // Edge verticale (sotto)
                if (j < grid_height - 1) {
                    uint32_t bottom_idx = (j + 1) * grid_width + i;
                    add_edge(compliance, idx, bottom_idx);
                }
                
                // Edge diagonali (shear constraints)
                if (i < grid_width - 1 && j < grid_height - 1) {
                    // Diagonale ↘
                    uint32_t diag1_idx = (j + 1) * grid_width + (i + 1);
                    add_edge(compliance, idx, diag1_idx);
                    
                    // Diagonale ↙
                    uint32_t diag2_idx = (j + 1) * grid_width + (i - 1);
                    if (i > 0) {
                        add_edge(compliance, idx, diag2_idx);
                    }
                }
                
                // // Bend constraints (salta un punto)
                // if (i < grid_width - 2) {
                //     uint32_t skip_idx = j * grid_width + (i + 2);
                //     add_edge(compliance, idx, skip_idx);
                // }
                
                // if (j < grid_height - 2) {
                //     uint32_t skip_idx = (j + 2) * grid_width + i;
                //     add_edge(compliance, idx, skip_idx);
                // }
            }
        }
    }
    
    void add_edge(Real compliance, uint32_t v1, uint32_t v2) {
        Real rest_length = distance(positions[v1], positions[v2]);
        edges.emplace_back(compliance, this, v1, v2, rest_length);
    }
    
    void create_mesh() {
        // Crea triangoli per la mesh di rendering
        std::vector<GLuint> indices;
        indices.reserve((grid_width - 1) * (grid_height - 1) * 6);
        
        for (uint32_t j = 0; j < grid_height - 1; ++j) {
            for (uint32_t i = 0; i < grid_width - 1; ++i) {
                uint32_t top_left = j * grid_width + i;
                uint32_t top_right = top_left + 1;
                uint32_t bottom_left = (j + 1) * grid_width + i;
                uint32_t bottom_right = bottom_left + 1;
                
                // Primo triangolo (top-left, top-right, bottom-left)
                indices.push_back(top_left);
                indices.push_back(top_right);
                indices.push_back(bottom_left);
                
                // Secondo triangolo (top-right, bottom-right, bottom-left)
                indices.push_back(top_right);
                indices.push_back(bottom_right);
                indices.push_back(bottom_left);
            }
        }
        
        mesh = ClothMesh(&positions, indices, edges);
    }
    
public:
    void update_aabb() {
        if (positions.empty()) return;
        
        Real3 min = positions[0];
        Real3 max = positions[0];
        
        for (const auto& pos : positions) {
            min = glm::min(min, pos);
            max = glm::max(max, pos);
        }
        
        aabb = AABB(min, max);
    }
    
    void translate(const Real3& translation) {
        for (auto& pos : positions) {
            pos += translation;
        }
        update_aabb();
    }
    
    void set_velocity(const Real3& velocity) {
        std::fill(velocities.begin(), velocities.end(), velocity);
    }
    
    void make_static() {
        std::fill(inv_masses.begin(), inv_masses.end(), 0.0);
    }
    
    // Fissa i vertici lungo un bordo (es: top row per appendere il cloth)
    void fix_border(Border border) {
        switch (border) {
            case Border::TOP:
                // for (uint32_t i = 0; i < grid_width; ++i) {
                //     inv_masses[i] = 0.0; // Prima riga
                // }
                inv_masses[0] = 0.0;
                inv_masses[grid_width-1] = 0.0;
                break;
            case Border::BOTTOM:
                for (uint32_t i = 0; i < grid_width; ++i) {
                    uint32_t idx = (grid_height - 1) * grid_width + i;
                    inv_masses[idx] = 0.0;
                }
                break;
            case Border::LEFT:
                for (uint32_t j = 0; j < grid_height; ++j) {
                    inv_masses[j * grid_width] = 0.0;
                }
                break;
            case Border::RIGHT:
                for (uint32_t j = 0; j < grid_height; ++j) {
                    uint32_t idx = j * grid_width + (grid_width - 1);
                    inv_masses[idx] = 0.0;
                }
                break;
        }
    }
    
    void draw() {
        mesh.drawWireframe();
        // mesh.drawPoints(); // Opzionale: mostra i punti
    }
    
    uint32_t num_vertices() const { return positions.size(); }
    uint32_t num_edges() const { return edges.size(); }
    
    // Utility per accedere alla griglia
    uint32_t get_vertex_index(uint32_t i, uint32_t j) const {
        return j * grid_width + i;
    }
    
    Real3 get_vertex_position(uint32_t i, uint32_t j) const {
        return positions[get_vertex_index(i, j)];
    }
};

