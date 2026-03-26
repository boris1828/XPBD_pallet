#pragma once

#include <glm/glm.hpp>
#include <glad/glad.h>

#include "scene.cpp"
#include "settings.cpp"
#include "types.h"

extern bool render_tearing;
extern Real tearing_stretch_percentage;

struct SpringRenderer 
{

    GLuint VAO, VBO;
    std::vector<Real3_Color> vertices;

    SpringRenderer() = default;
    
    ~SpringRenderer() 
    {
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
    }

    void init(Scene &scene) 
    {
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);

        buildVertices(scene.constraints);

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3_Color) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 4, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)(3 * sizeof(Real)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

    void buildVertices(std::vector<SpringConstraint> constraints) {
        vertices.clear();
        for (SpringConstraint &cons : constraints) 
        {
            Real3 v1 = cons.obj1->positions[cons.v1]; 
            Real3 v2 = cons.obj2->positions[cons.v2];
            Real3_Color vc1 = {v1.x, v1.y, v1.z, 0.0, 1.0, 0.0, 0.3};
            Real3_Color vc2 = {v2.x, v2.y, v2.z, 0.0, 1.0, 0.0, 0.3};
            vertices.push_back(vc1);
            vertices.push_back(vc2);
        }
    }

    void draw(Scene &scene) {
        buildVertices(scene.constraints);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3_Color) * vertices.size(), vertices.data());

        glDrawArrays(GL_LINES, 0, vertices.size());
        glBindVertexArray(0);
    }
};

struct FixedRigidSpringRenderer 
{

    GLuint VAO, VBO;
    std::vector<Real3_Color> vertices;

    FixedRigidSpringRenderer() = default;
    
    ~FixedRigidSpringRenderer() 
    {
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
    }

    void init(Scene &scene) 
    {
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);

        buildVertices(scene.fixed_rigid_constraints);

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3_Color) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 4, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)(3 * sizeof(Real)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

    void buildVertices(std::vector<FixedRigidSpringConstraint> constraints) 
    {
        vertices.clear();
        for (FixedRigidSpringConstraint &cons : constraints) {
            Real3 v1 = body_to_world(cons.body_attach, cons.box->position, cons.box->orientation);
            Real3 v2 = cons.world_attach;
            Real3_Color vc1 = {v1.x, v1.y, v1.z, 0.0, 1.0, 0.0, 1.0};
            Real3_Color vc2 = {v2.x, v2.y, v2.z, 0.0, 1.0, 0.0, 1.0};
            vertices.push_back(vc1);
            vertices.push_back(vc2);
        }
    }

    void draw(Scene &scene) 
    {
        buildVertices(scene.fixed_rigid_constraints);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3_Color) * vertices.size(), vertices.data());

        glDrawArrays(GL_LINES, 0, vertices.size());
        glBindVertexArray(0);
    }
};

struct RigidSpringRenderer 
{

    GLuint VAO, VBO;
    std::vector<Real3_Color> vertices;

    RigidSpringRenderer() = default;
    
    ~RigidSpringRenderer() 
    {
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
    }

    void init(Scene &scene) 
    {
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
        
        buildVertices(scene.rigid_constraints);

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3_Color) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 4, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)(3 * sizeof(Real)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

    void buildVertices(std::vector<RigidSpringConstraint> constraints) 
    {
        vertices.clear();
        for (RigidSpringConstraint &cons : constraints) 
        {
            if (cons.active == false) continue;

            Real r = 0.0;
            Real g = 1.0;
            Real b = 0.0;

            Real3 v1 = body_to_world(cons.r1, cons.b1->position, cons.b1->orientation);
            Real3 v2 = body_to_world(cons.r2, cons.b2->position, cons.b2->orientation);

            if (render_tearing )
            {
                Real curr_length    = getLength(cons);
                Real stretch        = curr_length - cons.rest_length;
                Real tear_threshold = cons.rest_length * tearing_stretch_percentage;

                Real t = stretch / tear_threshold;
                t = glm::clamp(t, 0.0, 1.0);

                r = t;
                g = 1.0 - t;
            }

            Real3_Color vc1 = {v1.x - 0.03, v1.y + 0.03, v1.z + 0.03, r, g, b, 1.0};
            Real3_Color vc2 = {v2.x - 0.03, v2.y + 0.03, v2.z + 0.03, r, g, b, 1.0};
            vertices.push_back(vc1);
            vertices.push_back(vc2);
        }
    }

    void draw(Scene &scene) 
    {
        buildVertices(scene.rigid_constraints);

        glBindVertexArray(VAO);

        glLineWidth(3.0f);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3_Color) * vertices.size(), vertices.data());

        glDrawArrays(GL_LINES, 0, vertices.size());

        glLineWidth(1.0f);

        glBindVertexArray(0);
    }
};

// struct NormalRenderer

void NormalRenderer::init() {
    normals.insert(normals.end(), 6, Real3_Color{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0});

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Real3_Color) * normals.size(), normals.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 4, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)(3 * sizeof(Real)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void NormalRenderer::buildNormals(const Quat& orientation, const Real3& position, Real size) {
    normals.clear();
    std::array<Real3, 3> bnormals = quat_to_axes(orientation);

    for (const auto& n : bnormals) {
        Real3 p1 = position;
        Real3 p2 = position + size * n;
        normals.push_back({p1.x, p1.y, p1.z, 1.0, 0.0, 0.0, 1.0});
        normals.push_back({p2.x, p2.y, p2.z, 1.0, 0.0, 0.0, 1.0});
    }
}
    
NormalRenderer::~NormalRenderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void NormalRenderer::draw(RigidBox *box) {
    Real size = glm::min(box->size.x, box->size.y);
    size      = glm::min(size, box->size.z) * 0.5;
    buildNormals(box->orientation, box->position, size);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3_Color) * normals.size(), normals.data());

    glDrawArrays(GL_LINES, 0, normals.size());
    glBindVertexArray(0);
}

// END struct NormalRenderer

struct ManifoldRenderer {
    GLuint VAO, VBO;
    std::vector<Real3_Color> vertices;

    ManifoldRenderer() = default;

    ~ManifoldRenderer() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    void init() {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3_Color) * 16, nullptr, GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 4, GL_DOUBLE, GL_FALSE, 7 * sizeof(Real), (void*)(3 * sizeof(Real)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

    void buildVertices(const RigidCollisionInfo& coll) {
        vertices.clear();

        for (int i = 0; i < coll.manifold_size; i++) {
            Real3 p        = coll.manifold[i];
            Real3_Color vc = {p.x, p.y, p.z, 1.0, 0.0, 0.0, 0.5}; 
            vertices.push_back(vc);
        }
    }

    void draw(const RigidCollisionInfo& coll) {
        if (!coll.intersecting || coll.manifold_size == 0) return;

        buildVertices(coll);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3_Color) * vertices.size(), vertices.data());

        glLineWidth(2.0f);
        glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size());
        glDrawArrays(GL_LINE_LOOP, 0, vertices.size());

        glBindVertexArray(0);
    }
};
