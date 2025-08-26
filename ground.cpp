
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

struct Ground {
    Ground() = default; 
    ~Ground();

    void draw();
    void init(float size = 10.0f, float yPos = -2.0f, glm::vec4 color = glm::vec4(1.0f, 1.0f, 1.0f, 0.1f));

    void drawGrid();
    void initGrid(float size = 10.0f, float yPos = -2.0f, glm::vec4 lineColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.3f));

    GLuint VAO, VBO, EBO;
    GLuint gridVAO, gridVBO;
    int indexCount;
    int gridVertexCount;
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

void Ground::initGrid(float size, float yPos, glm::vec4 lineColor) {
    float half = size / 2.0f;
    std::vector<float> gridVertices;

    for (float x = -half; x <= half; x += 1.0f) {
        // Linea parallela all'asse Z
        gridVertices.push_back(x);
        gridVertices.push_back(yPos + 0.01f); // leggermente sopra per evitare z-fighting
        gridVertices.push_back(-half);
        gridVertices.push_back(lineColor.r);
        gridVertices.push_back(lineColor.g);
        gridVertices.push_back(lineColor.b);
        gridVertices.push_back(lineColor.a);

        gridVertices.push_back(x);
        gridVertices.push_back(yPos + 0.01f);
        gridVertices.push_back(half);
        gridVertices.push_back(lineColor.r);
        gridVertices.push_back(lineColor.g);
        gridVertices.push_back(lineColor.b);
        gridVertices.push_back(lineColor.a);
    }

    // Se vuoi anche le linee parallele all'asse X
    for (float z = -half; z <= half; z += 1.0f) {
        gridVertices.push_back(-half);
        gridVertices.push_back(yPos + 0.01f);
        gridVertices.push_back(z);
        gridVertices.push_back(lineColor.r);
        gridVertices.push_back(lineColor.g);
        gridVertices.push_back(lineColor.b);
        gridVertices.push_back(lineColor.a);

        gridVertices.push_back(half);
        gridVertices.push_back(yPos + 0.01f);
        gridVertices.push_back(z);
        gridVertices.push_back(lineColor.r);
        gridVertices.push_back(lineColor.g);
        gridVertices.push_back(lineColor.b);
        gridVertices.push_back(lineColor.a);
    }

    gridVertexCount = gridVertices.size() / 7;

    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);

    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(float), gridVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}


void Ground::drawGrid() {
    glBindVertexArray(gridVAO);
    glDrawArrays(GL_LINES, 0, gridVertexCount);
    glBindVertexArray(0);
}

void Ground::draw() {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

Ground::~Ground() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
}