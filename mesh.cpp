#pragma once

#include <cstdint>
#include <vector>
#include <set>

#include <glm/glm.hpp>
#include <glad/glad.h>

#include "types.h"
#include "constraint.cpp"

#include <stdio.h>

Real tetra_volume(Real3 &x1, Real3 &x2, Real3 &x3, Real3 &x4) 
{
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

struct TetraMesh 
{
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

struct BoxMesh 
{
    std::vector<Real3>* vertices;
    GLuint VAO, VBO, EBO_edges, EBO_faces;  // Two separate EBOs
    std::vector<GLuint> edgeIndices;
    std::vector<GLuint> faceIndices;

    BoxMesh() = default;
    
    BoxMesh(std::vector<Real3> *vs) : vertices(vs) 
    {
        buildEdgeIndices();
        buildFaceIndices();
        
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO_edges);
        glGenBuffers(1, &EBO_faces);
        
        glBindVertexArray(VAO);
        
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3) * vertices->size(), 
                     vertices->data(), GL_DYNAMIC_DRAW);
        
        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Real3), (void*)0);
        glEnableVertexAttribArray(0);
        
        // Setup edge EBO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_edges);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * edgeIndices.size(), 
                     edgeIndices.data(), GL_STATIC_DRAW);
        
        glBindVertexArray(0);
        
        // Setup face EBO (upload separately)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_faces);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * faceIndices.size(), 
                     faceIndices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
    
    void clear() 
    {
        if (EBO_faces) glDeleteBuffers(1, &EBO_faces);
        if (EBO_edges) glDeleteBuffers(1, &EBO_edges);
        if (VBO)       glDeleteBuffers(1, &VBO);
        if (VAO)       glDeleteVertexArrays(1, &VAO);
    }
    
    void buildEdgeIndices() 
    {
        edgeIndices = {
            0, 1, 1, 2, 2, 3, 3, 0,
            4, 5, 5, 6, 6, 7, 7, 4,
            0, 4, 1, 5, 2, 6, 3, 7
        };
    }
    
    void buildFaceIndices() 
    {
        faceIndices = {
            0, 1, 2,  2, 3, 0,  // Front
            4, 7, 6,  6, 5, 4,  // Back
            0, 3, 7,  7, 4, 0,  // Left
            1, 5, 6,  6, 2, 1,  // Right
            3, 2, 6,  6, 7, 3,  // Top
            0, 4, 5,  5, 1, 0   // Bottom
        };
    }
    
    void drawSolid() 
    {
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_faces);  
        glDrawElements(GL_TRIANGLES, faceIndices.size(), GL_UNSIGNED_INT, 0);
    }
    
    void drawWireframe() 
    {
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_edges); 
        glLineWidth(1.0f);
        glDrawElements(GL_LINES, edgeIndices.size(), GL_UNSIGNED_INT, 0);
    }
};

struct TriangleMesh {

    std::vector<Real3>  *vertices;
    std::vector<GLuint> triangleIndices;
    GLuint VAO, VBO, EBO;

    TriangleMesh() = default;

    TriangleMesh(std::vector<Real3> *vs, const std::vector<GLuint>& indices)
        : vertices(vs), triangleIndices(indices) {

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3) * vertices->size(), vertices->data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Real3), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * triangleIndices.size(), triangleIndices.data(), GL_DYNAMIC_DRAW);

        glBindVertexArray(0);
    }

    void drawSolid() {
        glBindVertexArray(VAO);

        glDrawElements(GL_TRIANGLES, triangleIndices.size(), GL_UNSIGNED_INT, 0);
    }

    void drawWireframe() {
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());
        
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, triangleIndices.size(), GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
};

struct ClothMesh {
    GLuint VAO = 0, VBO = 0, EBO = 0;
    std::vector<Real3>* vertices = nullptr;
    std::vector<GLuint> triangleIndices;  // Rinominato per consistenza
    
    ClothMesh() = default;
    
    ClothMesh(std::vector<Real3>* vs, 
              const std::vector<GLuint>& inds,
              const std::vector<ClothEdge>& edges)
        : vertices(vs), triangleIndices(inds)  // Usa triangleIndices
    {
        setupMesh();
    }
    
    void setupMesh() {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        
        glBindVertexArray(VAO);
        
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Real3) * vertices->size(), 
                     vertices->data(), GL_DYNAMIC_DRAW);
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * triangleIndices.size(),
                     triangleIndices.data(), GL_DYNAMIC_DRAW);
        
        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Real3), (void*)0);
        glEnableVertexAttribArray(0);
        
        glBindVertexArray(0);
    }
    
    void drawWireframe() {
        if (VAO == 0) return;
        
        glBindVertexArray(VAO);
        
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());
        
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, triangleIndices.size(), GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        
        glBindVertexArray(0);
    }
    
    void drawSolid() {
        if (VAO == 0) return;
        
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());
        
        glDrawElements(GL_TRIANGLES, triangleIndices.size(), GL_UNSIGNED_INT, 0);
        
        glBindVertexArray(0);
    }
    
    void drawPoints() {
        if (VAO == 0) return;
        
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Real3) * vertices->size(), vertices->data());
        
        glPointSize(5.0f);
        glDrawArrays(GL_POINTS, 0, vertices->size());
        glPointSize(1.0f);
        
        glBindVertexArray(0);
    }
    
    ~ClothMesh() {
    //     if (VBO != 0) glDeleteBuffers(1, &VBO);
    //     if (EBO != 0) glDeleteBuffers(1, &EBO);
    //     if (VAO != 0) glDeleteVertexArrays(1, &VAO);
    }
};
