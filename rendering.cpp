#pragma once

#include <glm/glm.hpp>
#include <glad/glad.h>

#include "scene.cpp"
#include "types.cpp"

struct Real3_Color {
    Real x, y, z, r, g, b, a;
};

struct SpringRenderer {

    GLuint VAO, VBO, EBO;
    std::vector<Real3_Color> vertices;

    SpringRenderer() = default;
    
    ~SpringRenderer() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

    void init(Scene &scene) {
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
        for (SpringConstraint &cons : constraints) {
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