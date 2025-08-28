#pragma once

#include <cstdint>
#include <vector>
#include <set>

#include <glm/glm.hpp>
#include <glad/glad.h>

#include "types.cpp"
#include "constraint.cpp"

#include <stdio.h>

Real tetra_volume(Real3 &x1, Real3 &x2, Real3 &x3, Real3 &x4) {
    Real3 v1 = x2 - x1;
    Real3 v2 = x3 - x1;
    Real3 v3 = x4 - x1;

    Real3 cross = glm::cross(v1, v2);
    Real dot    = glm::dot(cross, v3);
    Real volume = std::abs(dot) / 6.0;

    return volume;
}

inline Real distance(Real3 &p1, Real3 &p2) {
    return glm::length(p2 - p1);
}

struct TetraMesh {
    std::vector<Real3> normalVertices;
    std::vector<Real3>* vertices;
    GLuint VAO, VBO, EBO;
    GLuint normalVAO, normalVBO;

    std::vector<GLuint> triangleIndices;
    std::vector<GLuint> edgeIndices;

    TetraMesh() = default;

    TetraMesh(std::vector<Real3> *vs, std::vector<Tetrahedron> &tetras, std::vector<Edge> &edges) :
        vertices(vs), normalVertices() {

        buildEdgeIndices(edges);
        buildTriangleIndices(tetras);

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3) * vertices->size(), vertices->data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Real3), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * edgeIndices.size(), edgeIndices.data(), GL_DYNAMIC_DRAW);

        // normals
        updateNormals(tetras);
        glGenVertexArrays(1, &normalVAO);
        glGenBuffers(1, &normalVBO);

        glBindVertexArray(normalVAO);

        glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
        glBufferData(GL_ARRAY_BUFFER, normalVertices.size() * sizeof(Real3), normalVertices.data(), GL_DYNAMIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Real3), (void*)0);

        glBindVertexArray(0);
    }

    void buildTriangleIndices(std::vector<Tetrahedron> &tetras) {
        // TODO
    }

    void buildEdgeIndices(std::vector<Edge> &edges) {
        for (auto& e : edges) {
            edgeIndices.push_back(e.v1); 
            edgeIndices.push_back(e.v2);
        }
    }

    void setEdgesToDraw(const std::vector<Edge>& edgesToDraw) {
        std::vector<GLuint> newIndices;
        newIndices.reserve(edgesToDraw.size() * 2);

        for (auto& e : edgesToDraw) {
            newIndices.push_back(e.v1);
            newIndices.push_back(e.v2);
        }

        edgeIndices = newIndices;

        glBindVertexArray(VAO); 

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    sizeof(GLuint) * edgeIndices.size(),
                    edgeIndices.data(),
                    GL_DYNAMIC_DRAW);

        glBindVertexArray(0);
    }

    void drawSolid() {
        // TODO
    }

    void drawWireframe() {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());

        glLineWidth(1.0f);
        glDrawElements(GL_LINES, edgeIndices.size(), GL_UNSIGNED_INT, 0);
    }

    void drawPoints() {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());

        glPointSize(10.0f); 
        glDrawArrays(GL_POINTS, 0, vertices->size());
    }

    void drawFaceNormals(std::vector<Tetrahedron> &tetras) {
        updateNormals(tetras);

        glBindVertexArray(normalVAO);

        glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, normalVertices.size() * sizeof(Real3), normalVertices.data());

        glLineWidth(1.0f);
        glDrawArrays(GL_LINES, 0, normalVertices.size());
    }

    void updateNormals(std::vector<Tetrahedron> &tetras) {

        normalVertices.clear();
        for (const auto& t : tetras) {
            Real3 p1 = vertices->operator[](t.vs[0]);
            Real3 p2 = vertices->operator[](t.vs[1]);
            Real3 p3 = vertices->operator[](t.vs[2]);
            Real3 p4 = vertices->operator[](t.vs[3]);

            Real length = 0.5f;

            auto addNormal = [&](Real3 a, Real3 b, Real3 c, Real3 opp) {
                Real3 normal = glm::normalize(glm::cross(b - a, c - a));
                Real3 center = (a + b + c) / 3.0;

                Real3 toOpposite = opp - center;
                if (glm::dot(normal, toOpposite) > 0.0) normal = -normal;

                normalVertices.push_back(center);                  
                normalVertices.push_back(center + normal * length);
            };

            addNormal(p1, p2, p3, p4);
            addNormal(p1, p3, p4, p2);
            addNormal(p1, p2, p4, p3);
            addNormal(p2, p3, p4, p1);
        }
    }

};


