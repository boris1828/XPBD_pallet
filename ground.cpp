
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

class Ground {
public:
    Ground() = default; 
    ~Ground();

    void draw();
    void init(float size = 100.0f, float yPos = -2.0f, glm::vec4 color = glm::vec4(0.0f, 0.6f, 0.0f, 0.3f));
private:
    GLuint VAO, VBO, EBO;
    int indexCount;
};


void Ground::init(float size, float yPos, glm::vec4 color) {

    float half = size / 2.0f;
    float vertices[] = {
        -half, yPos, -half, color.r, color.g, color.b, color.a,
         half, yPos, -half, color.r, color.g, color.b, color.a,
         half, yPos,  half, color.r, color.g, color.b, color.a,
        -half, yPos,  half, color.r, color.g, color.b, color.a
    };

    unsigned int indices[] = {
        0, 1, 2,
        2, 3, 0
    };
    indexCount = 6;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Posizione (3 float)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Colore (4 float)
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

Ground::~Ground() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void Ground::draw() {

    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    // glDisable(GL_BLEND);
}
