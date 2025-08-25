#include <glad/glad.h>

const char* vertexShaderSource = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor; // Per il ground

out vec4 vertexColor; // Varying verso il fragment shader

uniform mat4 MVP;

void main() {
    gl_Position = MVP * vec4(aPos, 1.0);
    vertexColor = aColor; // Passa il colore dal VBO
}
)glsl";

const char* fragmentShaderSource = R"glsl(
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(1.0, 0.5, 0.2, 1.0); // arancio
}
)glsl";

const char* groundFragmentShaderSource = R"glsl(
#version 330 core
in vec4 vertexColor;
out vec4 FragColor;
void main() {
    FragColor = vertexColor; // Colore dal VBO
}
)glsl";

bool compile_shader(const char* source, GLenum shader_type, unsigned int &vertexShader) {
    vertexShader = glCreateShader(shader_type);
    glShaderSource(vertexShader, 1, &source, NULL);
    glCompileShader(vertexShader);
    // Controllo errori compilazione
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "Errore compilazione shader:\n" << infoLog << std::endl;
        return false;
    }
    return true;
}


bool init_shaders(unsigned int &objectProgram, unsigned int &groundProgram) {

    int success;
    char infoLog[512];

    unsigned int vertexShader;
    if (!compile_shader(vertexShaderSource, GL_VERTEX_SHADER, vertexShader))
        return false;

    unsigned int fragmentShader;
    if (!compile_shader(fragmentShaderSource, GL_FRAGMENT_SHADER, fragmentShader))
        return false;

    unsigned int groundFragmentShader;
    if (!compile_shader(groundFragmentShaderSource,GL_FRAGMENT_SHADER, groundFragmentShader))
        return false;

    // --- Programma per oggetti
    objectProgram = glCreateProgram();
    glAttachShader(objectProgram, vertexShader);
    glAttachShader(objectProgram, fragmentShader);
    glLinkProgram(objectProgram);
    glGetProgramiv(objectProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(objectProgram, 512, NULL, infoLog);
        std::cerr << "Errore linking object shader program:\n" << infoLog << std::endl;
        return false;
    }

    // --- Programma per ground
    groundProgram = glCreateProgram();
    glAttachShader(groundProgram, vertexShader);
    glAttachShader(groundProgram, groundFragmentShader);
    glLinkProgram(groundProgram);
    glGetProgramiv(groundProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(groundProgram, 512, NULL, infoLog);
        std::cerr << "Errore linking ground shader program:\n" << infoLog << std::endl;
        return false;
    }

    // Puoi eliminare gli shader ora
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteShader(groundFragmentShader);

    return true;
}