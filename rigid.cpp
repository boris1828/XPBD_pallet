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

extern bool render_tearing;

struct RigidBox;


inline Quat quat_conjugate(const Quat& q) 
{
    return Quat(-q.x, -q.y, -q.z, q.w);
}

inline Quat quat_multiplication(const Quat &p, const Quat &q) 
{
    // p = (px,py,pz,pw), q = (qx,qy,qz,qw)
    Real px = p.x, py = p.y, pz = p.z, pw = p.w;
    Real qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    Quat r;
    r.x = pw*qx + px*qw + py*qz - pz*qy;
    r.y = pw*qy - px*qz + py*qw + pz*qx;
    r.z = pw*qz + px*qy - py*qx + pz*qw;
    r.w = pw*qw - px*qx - py*qy - pz*qz;
    return r;
}

inline Real3x3 quat_to_rotmat(const Quat& q) 
{
    Real x = q.x;
    Real y = q.y;
    Real z = q.z;
    Real w = q.w;

    Real xx = x * x;
    Real yy = y * y;
    Real zz = z * z;
    Real xy = x * y;
    Real xz = x * z;
    Real yz = y * z;
    Real wx = w * x;
    Real wy = w * y;
    Real wz = w * z;

    return Real3x3(
        1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy),
        2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)
    );
}

inline std::array<Real3, 3> quat_to_axes(const Quat& q) 
{
    Real x = q.x;
    Real y = q.y;
    Real z = q.z;
    Real w = q.w;

    Real xx = x * x;
    Real yy = y * y;
    Real zz = z * z;
    Real xy = x * y;
    Real xz = x * z;
    Real yz = y * z;
    Real wx = w * x;
    Real wy = w * y;
    Real wz = w * z;

    std::array<Real3, 3> axes;

    axes[0] = Real3(1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy));
    axes[1] = Real3(    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx));
    axes[2] = Real3(    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy));

    axes[0] = glm::normalize(axes[0]);
    axes[1] = glm::normalize(axes[1]);
    axes[2] = glm::normalize(axes[2]);

    return axes;
}

inline Real3 body_to_world(const Real3& p_body, const Real3& pos, const Quat& q) 
{
    Real3x3 R = quat_to_rotmat(q);
    return R * p_body + pos;
}

inline Real3 world_to_body(const Real3& p_world, const Real3& pos, const Quat& q) 
{
    Real3x3 R = quat_to_rotmat(q);
    return glm::transpose(R) * (p_world - pos);
}

inline Real3 world_to_body(const Real3& p_world, const Quat& q) 
{
    Real3x3 R = quat_to_rotmat(q);
    return glm::transpose(R) * p_world;
}

struct RigidBox 
{
    Real3   position;
    Real3   old_position;
    Real3   velocity;
    Real    mass;
    Real    inv_mass;

    Quat    orientation;
    Quat    old_orientation;
    Real3   angular_velocity;
    Real3x3 inertia_tensor;
    Real3x3 inv_inertia_tensor;

    std::vector<Real3> world_vertices;
    std::vector<Real3> body_vertices;
    Real3   size;
    BoxMesh mesh;
    AABB    aabb;

    bool is_static;

    RigidBox(Real3 pos, Real3 size, Real mass)
        : position(pos), 
          velocity(0.0), 
          mass(mass), 
          inv_mass(mass > 0.0 ? 1.0 / mass : 0.0), 
          orientation(0.0, 0.0, 0.0, 1.0), 
          angular_velocity(0.0), 
          size(size),
          aabb(pos - size * 0.5, pos + size * 0.5),
          is_static(false)
    {

        body_vertices = {
            Real3(-size.x/2, -size.y/2, -size.z/2),
            Real3(-size.x/2, -size.y/2,  size.z/2),
            Real3( size.x/2, -size.y/2,  size.z/2),
            Real3( size.x/2, -size.y/2, -size.z/2),
            Real3(-size.x/2,  size.y/2, -size.z/2),
            Real3(-size.x/2,  size.y/2,  size.z/2),
            Real3( size.x/2,  size.y/2,  size.z/2),
            Real3( size.x/2,  size.y/2, -size.z/2),
        };

        for (Real3 &v : body_vertices) {
            world_vertices.push_back(v + pos);
        }

        Real ix = (1.0 / 12.0) * mass * (size.y * size.y + size.z * size.z);
        Real iy = (1.0 / 12.0) * mass * (size.x * size.x + size.z * size.z);
        Real iz = (1.0 / 12.0) * mass * (size.x * size.x + size.y * size.y);
        inertia_tensor = Real3x3(
            ix, 0.0, 0.0,
            0.0, iy, 0.0,
            0.0, 0.0, iz
        );
        inv_inertia_tensor = glm::inverse(inertia_tensor);

        mesh = BoxMesh(&world_vertices);
    }

    RigidBox(RigidBox&& other) noexcept
        : position(other.position),
          old_position(other.old_position),
          velocity(other.velocity),
          mass(other.mass),
          inv_mass(other.inv_mass),
          orientation(other.orientation),
          angular_velocity(other.angular_velocity),
          old_orientation(other.old_orientation),
          inertia_tensor(other.inertia_tensor),
          inv_inertia_tensor(other.inv_inertia_tensor),
          size(other.size),
          aabb(other.aabb),
          world_vertices(std::move(other.world_vertices)),
          body_vertices(std::move(other.body_vertices)),
          mesh(std::move(other.mesh)),
          is_static(other.is_static)
    {
        mesh.vertices = &world_vertices;
    }

    RigidBox& operator=(RigidBox&& other) noexcept {
        if (this != &other) {
            position           = other.position;
            old_position       = other.old_position;
            velocity           = other.velocity;
            mass               = other.mass;
            inv_mass           = other.inv_mass;
            orientation        = other.orientation;
            old_orientation    = other.old_orientation;
            angular_velocity   = other.angular_velocity;
            inertia_tensor     = other.inertia_tensor;
            inv_inertia_tensor = other.inv_inertia_tensor;
            aabb               = other.aabb;
            size               = other.size;
            is_static          = other.is_static;

            world_vertices     = std::move(other.world_vertices);
            body_vertices      = std::move(other.body_vertices);
            mesh               = std::move(other.mesh);

            mesh.vertices      = &world_vertices;
        }
        return *this;
    }

    Real generalized_inverse_mass(Real3 pb, Real3 nb) const {

        if (is_static) return 0.0;

        Real3 r_cross_n = glm::cross(pb, nb);
        Real w          = inv_mass + glm::dot(r_cross_n, inv_inertia_tensor * r_cross_n);
        return w;
    }

    void clear() 
    {
        mesh.clear();
    }

    void update(Real delta_t, Real3 gravity) 
    {
        if (is_static) return;

        old_position    = position;
        old_orientation = orientation;

        Real3 acceleration  = gravity;
        velocity           += acceleration * delta_t;
        position           += velocity     * delta_t;

        #ifdef OLD_IMPL

        Real3 Iw          = inertia_tensor * angular_velocity;
        Real3 gyro        = glm::cross(angular_velocity, Iw);
        angular_velocity += delta_t * (inv_inertia_tensor * (-gyro));
        Quat omega_q(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0);
        orientation += 0.5 * delta_t * (quat_multiplication(omega_q, orientation));
        orientation  = glm::normalize(orientation);

        #else

        Real3x3 R                 = quat_to_rotmat(orientation); 
        Real3x3 inertia_world     = R * inertia_tensor     * glm::transpose(R);
        Real3x3 inv_inertia_world = R * inv_inertia_tensor * glm::transpose(R);

        Real3 Iw   = inertia_world * angular_velocity;
        Real3 gyro = glm::cross(angular_velocity, Iw);
        angular_velocity += delta_t * (inv_inertia_world * (-gyro));

        Quat omega_q(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0);
        orientation += 0.5 * delta_t * (quat_multiplication(omega_q, orientation));
        orientation = glm::normalize(orientation);

        #endif

        update_world_vertices();
        update_aabb();
    }

    Real min_x() const 
    {
        Real min_x = world_vertices[0].x;
        for (const Real3& v : world_vertices) 
            if (v.x < min_x) min_x = v.x;
        return min_x;
    }

    Real max_y() const 
    {
        Real max_y = world_vertices[0].y;
        for (const Real3& v : world_vertices) if (v.y > max_y) max_y = v.y;
        return max_y;
    }

    void update_velocities(Real delta_t) 
    {
        if (is_static) return;

        velocity = (position - old_position) / delta_t;

        Quat dquat       = quat_multiplication(orientation, quat_conjugate(old_orientation));
        angular_velocity = (2.0 / delta_t) * Real3(dquat.x, dquat.y, dquat.z);
        angular_velocity = dquat.w >= 0.0? angular_velocity : -angular_velocity;
    }
    
    void update_world_vertices() 
    {
        world_vertices.clear();
        world_vertices.reserve(body_vertices.size());
        for (const Real3& v_body : body_vertices) {
            world_vertices.push_back(body_to_world(v_body, position, orientation));
        }
    }

    void update_aabb() 
    {
        Real3 min_v = world_vertices[0];
        Real3 max_v = world_vertices[0];
        for (const Real3& v : world_vertices) {
            min_v = glm::min(min_v, v);
            max_v = glm::max(max_v, v);
        }
        aabb.min = min_v;
        aabb.max = max_v;
    }

    void draw() 
    {
        if (render_tearing) mesh.drawSolid();
        else                mesh.drawWireframe();
    }

    void rotate(const Quat& q_rotation) {
        orientation = quat_multiplication(orientation, q_rotation);
        orientation = glm::normalize(orientation);
        update_world_vertices();
        update_aabb();
    }

    void translate(const Real3& offset) {
        position += offset;
        update_world_vertices();
        update_aabb();
    }

    void make_static() {
        is_static = true;
        inv_mass  = 0.0;
    }
};

// ====================================
// Rendering Cool Stuff
// ====================================

struct NormalRenderer 
{
    GLuint VAO = 0, VBO = 0;
    std::vector<Real3_Color> normals;

    NormalRenderer() = default;
    ~NormalRenderer();
    void init();
    void buildNormals(const Quat& orientation, const Real3& position, Real size);
    void draw(RigidBox *box);
};

struct SceneObject 
{
    TriangleMesh mesh;
    std::vector<Real3> vertices;

    SceneObject() = default;

    SceneObject(const std::vector<Real3>& vs, const std::vector<GLuint>& indices) {
        init(vs, indices);
    }

    SceneObject(SceneObject&& other) noexcept
        : mesh(std::move(other.mesh)),
          vertices(std::move(other.vertices))
    {
        mesh.vertices = &vertices;
    }

    SceneObject& operator=(SceneObject&& other) noexcept {
        if (this != &other) {
            mesh     = std::move(other.mesh);
            vertices = std::move(other.vertices);
            mesh.vertices = &vertices;
        }
        return *this;
    }

    void init(const std::vector<Real3>& vs, const std::vector<GLuint>& indices) {
        vertices = vs;
        mesh     = TriangleMesh(&vertices, indices);
    }

    void translate(const Real3& offset) {
        for (Real3 &v : vertices) {
            v += offset;
        }
    }

    void draw() {
        mesh.drawWireframe();
    }
};

SceneObject load_scene_object_from_obj(const std::string& filename, Real scale_factor) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Impossibile aprire file: " + filename);
    }

    std::vector<Real3> vertices;
    std::vector<GLuint> indices;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            // Riga vertice
            float x, y, z;
            iss >> x >> y >> z;
            vertices.push_back(Real3(x, y, z) * scale_factor);
        } 
        else if (prefix == "f") {
            // Riga faccia (può essere triangolo o poligono)
            std::string token;
            std::vector<int> faceIndices;
            while (iss >> token) {
                // formato: v/t/n oppure v//n oppure v
                std::istringstream vt(token);
                std::string vStr;
                std::getline(vt, vStr, '/'); // prendo solo la parte prima di '/'
                int vi = std::stoi(vStr);
                faceIndices.push_back(vi - 1); // .obj è 1-based
            }

            // Triangolazione "fan" (f v1 v2 v3 v4 → (v1,v2,v3), (v1,v3,v4))
            for (size_t i = 1; i + 1 < faceIndices.size(); i++) {
                indices.push_back(faceIndices[0]);
                indices.push_back(faceIndices[i]);
                indices.push_back(faceIndices[i + 1]);
            }
        }
    }

    file.close();

    SceneObject obj(vertices, indices);

    return obj;
}

