#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <chrono>
#include <random>
#include <thread>
#include <string>
#include <sstream>
#include <unordered_map>
#include <cmath>

#include <fstream>
#include <sstream>
#include <iomanip>

#include <filesystem>
namespace fs = std::filesystem;

#include "shader.cpp"
#include "scene.cpp"
#include "types.cpp"
#include "xpbd.cpp"
#include "ground.cpp"
#include "XMLparser.cpp"
#include "constants.cpp"
#include "settings.cpp"

// Callback per ridimensionamento finestra
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

extern Real delta_t; 

extern unsigned int WIDTH;
extern unsigned int HEIGHT;
extern bool DO_VIDEO;
extern double targetFPS;

const std::chrono::duration<double, std::milli> targetFrameDuration(1000.0 / targetFPS);
std::chrono::steady_clock::time_point frameStart;
std::chrono::steady_clock::time_point simulationStart;

GLFWwindow* window         = nullptr;
unsigned int objectProgram = 0;
unsigned int groundProgram = 0;
Scene  scene;
Ground ground; 

extern Real wrap_compliance;

void clear_video_folder() {
    fs::path folder("..\\..\\video_frame");

    fs::create_directories(folder);

    for (const auto& entry : fs::directory_iterator(folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".ppm") {
            std::error_code ec; // per evitare eccezioni
            fs::remove(entry.path(), ec);
            if (ec) {
                std::cerr << "Errore cancellando " << entry.path() << ": " << ec.message() << "\n";
            }
        }
    }
}

bool graphics_init() {

    if (DO_VIDEO) clear_video_folder();

    // Inizializza GLFW
    if (!glfwInit()) {
        std::cerr << "Errore inizializzazione GLFW\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(WIDTH, HEIGHT, "XPBD", NULL, NULL);
    if (!window) {
        std::cerr << "Errore creazione finestra GLFW\n";
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);

    // Inizializza GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Errore inizializzazione GLAD\n";
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    ground.init();
    ground.initGrid();

    return init_shaders(objectProgram, groundProgram);
}

void graphics_close() {
    glDeleteProgram(objectProgram);
    glDeleteProgram(groundProgram);
    glfwTerminate();
}

// ############# LOOP #############

void loop_input() {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

void loop_init() {
    frameStart = std::chrono::high_resolution_clock::now();
    loop_input();
}

void loop_terminate() {
    glfwSwapBuffers(window);
    glfwPollEvents();

    // auto frameEnd = std::chrono::high_resolution_clock::now();
    // auto frameDuration = frameEnd - frameStart;

    // if (frameDuration < targetFrameDuration) {
    //     std::this_thread::sleep_for(targetFrameDuration - frameDuration);
    // }
}

// ############# RENDERING #############

glm::mat4 get_MVP(Real3 center = Real3(0.0)) {
    
    glm::mat4 model = glm::mat4(1.0f);
    model           = glm::rotate(model, glm::radians(0.0f), /*((float)glfwGetTime()),*/ glm::vec3(0.3f, 1.0f, 0.0f));

    glm::mat4 view = glm::lookAt(
        glm::vec3(center.x, center.y, 2.0),      // posizione camera
        glm::vec3(center.x, center.y, center.z), // guarda al centro
        glm::vec3(0.0f, 1.0f, 0.0f)              // up
    );
    glm::mat4 projection = glm::perspective(
        glm::radians(80.0f),
        ((float)WIDTH) / ((float)HEIGHT),
        0.1f,
        100.0f
    );
    glm::mat4 MVP = projection * view * model;
    return MVP;
}

void set_shader(unsigned int shaderProgram, glm::mat4 &MVP) {
    glUseProgram(shaderProgram);
    int mvpLoc = glGetUniformLocation(shaderProgram, "MVP");
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, glm::value_ptr(MVP));
}

void background(float r, float g, float b) {
    glClearColor(r, g, b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void rendering() {
    glm::mat4 MVP = get_MVP(scene.center());

    set_shader(objectProgram, MVP);
    background(0.1f, 0.3f, 0.2f);
    for (TetraObject &obj : scene.objects)
        obj.draw();

    set_shader(groundProgram, MVP);
    ground.draw();
    ground.drawGrid();

}

// ######################################################################

void attach_vertices(
        Scene &scene, 
        Index       obj1, Index       obj2, 
        VertexIndex v1,   VertexIndex v2, 
        Real rest_length = 0.0) {

    TetraObject &o1 = scene.getObject(obj1);
    TetraObject &o2 = scene.getObject(obj2);

    SpringConstraint con(wrap_compliance, &o1, &o2, v1, v2, rest_length);
    scene.addConstraint(con);
}

std::string find_single_file(const std::string& folder_path) {
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            return entry.path().string(); // ritorna il path completo
        }
    }
    throw std::runtime_error("Nessun file trovato in " + folder_path);
}

std::vector<std::string> find_files_in_folder(const std::string& folder_path) {
    std::vector<std::string> files;
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path().string());
        }
    }
    if (files.empty()) {
        throw std::runtime_error("Nessun file trovato in " + folder_path);
    }
    return files;
}

struct Box {
    Real3 position;
    Real3 size;
    Real  weight;
};

void load_schema(const std::string& schema_path) {
    try {

        XMLParser pallet_parser( find_single_file("..\\..\\" + schema_path + "\\Pallet"));
        XMLParser secondary_parser(find_single_file("..\\..\\" + schema_path + "\\SecondaryPackaging"));
        XMLParser schema_parser(find_single_file("..\\..\\" + schema_path + "\\PalletisingSchema"));

        std::vector<XMLParser> layer_parsers;
        auto layers_files = find_files_in_folder( "..\\..\\" + schema_path + "\\Layer");
        for (const auto& file : layers_files)
            layer_parsers.emplace_back(file);

        XMLNode pallet_XML  = pallet_parser.parse();
        XMLNode secondary_XML = secondary_parser.parse();
        XMLNode schema_XML  = schema_parser.parse();

        std::vector<XMLNode> layer_XMLs;
        for (auto& parser : layer_parsers)
            layer_XMLs.push_back(parser.parse());

        Real mult = 0.001;

        Real weight = mult * secondary_XML.find_first("Weight")->int_value;
        Real height = mult * secondary_XML.find_first("Height")->int_value;
        Real width  = mult * secondary_XML.find_first("Width")->int_value;
        Real length = mult * secondary_XML.find_first("Length")->int_value;

        std::vector<Box> boxes;

        int layer_num = 0;
        for (auto& layer_info : schema_XML["PalSchemaClass"][0]["LayerPaths"][0]["string"]) {
            std::string layer_file_name = layer_info.value;

            for (auto& layer_XML : layer_XMLs) {
                if (layer_XML.file_name != layer_file_name) continue;

                std::vector<XMLNode> boxes_position = layer_XML["LayerClass"][0]["SPDisposal"][0]["PalSchema_SPDisposalClass"];

                for (auto& box_pos : boxes_position) {
                    Real x = mult * box_pos.find_first("_x")->int_value;
                    Real z = mult * box_pos.find_first("_y")->int_value;
                    Real y = layer_num * height - 2.0;

                    if (box_pos.find_first("_rotation")->value == "true") 
                        boxes.push_back({Real3(x, y, z), Real3(width, height, length), weight});
                    else
                        boxes.push_back({Real3(x, y, z), Real3(length, height, width), weight});
                }
                break;
            }
            layer_num++;
            // if (layer_num > 0) break;
        }
        
        for (const auto& box : boxes) {
            scene.addObject(create_box(box.position, box.size.x, box.size.y, box.size.z));
            // for(VertexIndex vi=0; vi<8; vi++) {
            //     scene.getObject(scene.objects.size()-1).inv_masses[vi] = 1.0 / box.weight;
            // }
        }


    } catch (const std::exception& e) {
        std::cerr << "Error loading schema: " << e.what() << "\n";
    }
}

void wrap() {

    // AABB of all objects
    Real3 scene_min( std::numeric_limits<Real>::max());
    Real3 scene_max(-std::numeric_limits<Real>::max());

    for (const auto& obj : scene.objects) {
        scene_min = glm::min(scene_min, obj.aabb.min);
        scene_max = glm::max(scene_max, obj.aabb.max);
    }

    auto is_on_aabb_face = [&](const Real3& p, Real epsilon = 1e-5) -> bool {
        bool on_x_face = (std::abs(p.x - scene_min.x) < epsilon) || (std::abs(p.x - scene_max.x) < epsilon);
        bool on_y_face = (std::abs(p.y - scene_min.y) < epsilon) || (std::abs(p.y - scene_max.y) < epsilon);
        bool on_z_face = (std::abs(p.z - scene_min.z) < epsilon) || (std::abs(p.z - scene_max.z) < epsilon);
        return on_x_face /*|| on_y_face*/ || on_z_face;
    };

    for (Index o1i = 0; o1i < scene.objects.size(); o1i++) {
        for (Index o2i = o1i + 1; o2i < scene.objects.size(); o2i++) {
            TetraObject &obj1 = scene.objects[o1i];
            TetraObject &obj2 = scene.objects[o2i];
            for (VertexIndex vi1 = 0; vi1 < obj1.num_vertices(); vi1++) {
                for (VertexIndex vi2 = 0; vi2 < obj2.num_vertices(); vi2++) {
                    Real distance = glm::length(obj1.positions[vi1] - obj2.positions[vi2]);
                    if (distance < .3) {
                        if (is_on_aabb_face(obj1.positions[vi1])) {
                            attach_vertices(scene, o1i, o2i, vi1, vi2, distance);
                        }
                    }
                }
            }
        }
    }
}

void saveFrame(int width, int height, int frameNumber) {
    std::vector<unsigned char> pixels(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    std::ostringstream filename;
    filename << "..\\..\\video_frame\\frame_" << std::setw(5) << std::setfill('0') << frameNumber << ".ppm";

    std::ofstream out(filename.str(), std::ios::binary);
    out << "P6\n" << width << " " << height << "\n255\n";
    out.write(reinterpret_cast<char*>(pixels.data()), pixels.size());
    out.close();
}

struct KeyHash {
    std::size_t operator()(const std::pair<TetraObject*, VertexIndex>& k) const {
        return std::hash<TetraObject*>()(k.first) ^ (std::hash<VertexIndex>()(k.second) << 1);
    }
};

struct KeyEqual {
    bool operator()(const std::pair<TetraObject*, VertexIndex>& a,
                    const std::pair<TetraObject*, VertexIndex>& b) const {
        return a.first == b.first && a.second == b.second;
    }
};

void world_schema() {
    
    XPBD_init();

    uint64_t step = 0;
    Real time     = 0.0;

    try { load_schema("palleting_data\\A1625_12oz_6x4_ml_3Ls"); }
    catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return;
    }

    wrap();

    std::unordered_map<std::pair<TetraObject*, VertexIndex>, Real3, KeyHash, KeyEqual> positions;

    for (auto& obj : scene.objects) {
        for (VertexIndex vi = 0; vi < obj.num_vertices(); vi++) {
            if (std::abs(obj.positions[vi].y + 2.0) < 1e-6) { // Compare with epsilon
                obj.inv_masses[vi] = 0.0;
                positions[{&obj, vi}] = obj.positions[vi];
            }
        }
    }
    
    std::cout << "Loaded " << scene.objects.size()     << " objects.\n";
    std::cout << "Loaded " << scene.constraints.size() << " constraints.\n";
    std::cout << "Loaded " << positions.size()         << " fixed vertices.\n";

    simulationStart = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {

        time = step * delta_t;

        Real3 velocity(1.0, 0.0, 0.0);
        if (time < 2.0)       velocity *= time / 2.0;
        else if (time < 3.0)  velocity *= (3.0 - time) / 1.0;
        else                  velocity *= 0.0;

        for (const auto& [key, init_pos] : positions) {
            TetraObject* obj      = key.first;
            VertexIndex vi        = key.second;
            positions[{obj, vi}] += velocity * delta_t;
            obj->positions[vi]    = positions[{obj, vi}];
        }

        // if (time > 2.0) scene.removeAllConstraints();

        XPBD_step(scene);

        if (DO_VIDEO && step%10 == 0) {
            loop_init();
            rendering();
            saveFrame(WIDTH, HEIGHT, step/10);
            loop_terminate();
        } 
        else {
            loop_init();
            rendering();
            loop_terminate();
        }

        auto now        = std::chrono::high_resolution_clock::now();
        auto elapsed    = now - simulationStart;
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - simulationStart).count();
        double avg_time = static_cast<double>(elapsed_ms) / (step + 1);

        std::cout << "avg. step time:     " << avg_time <<  " ms \n";
        std::cout << "simul. time:        " << time     <<  " s\n\n";

        step++;
        if (time > 4.0) break;
    }
}

void world() {
    
    XPBD_init();

    uint64_t step = 0;
    Real time     = 0.0;

    scene.addObject(create_box(Real3(0.0, -2.0, -2.0), 1.0, 1.0, 1.0));
    scene.addObject(create_box(Real3(0.9, -1.0, -2.0), 1.0, 1.0, 1.0));

    while (!glfwWindowShouldClose(window)) {

        time = step * delta_t;

        loop_init();

        XPBD_step(scene);

        rendering(); 

        loop_terminate();

        step++;
    }
}

void world_collision_tester() {

    XPBD_init();

    uint64_t step = 0;
    Real time     = 0.0;

    // vertex-face collision
    // scene.addObject(create_tetrahedron(
    //                     Real3(0.0, -2.0, 0.0),
    //                     Real3(1.0, -2.0, 0.0),
    //                     Real3(0.0, -1.0, 0.0),
    //                     Real3(0.0, -2.0, 1.0)));
    // scene.addObject(create_tetrahedron(
    //                     Real3(0.3, -1.7, 0.3),
    //                     Real3(1.0, -0.5, 1.0),
    //                     Real3(1.5, -1.3, 0.5),
    //                     Real3(0.5, -1.3, 1.5)));

    scene.addObject(create_tetrahedron(
                        Real3(0.0, -1.0, 0.0),
                        Real3(1.0, -1.0, 0.0),
                        Real3(0.0, -0.0, 0.0),
                        Real3(0.0, -1.0, 1.0)));

    scene.addObject(create_tetrahedron(
                        Real3( 1.0, -0.5, -1.0),
                        Real3(-1.0, -0.5,  1.0),
                        Real3(-1.0, -0.0, -1.0),
                        Real3(-1.0, -1.0, -1.0)));

    TetraObject &obj1 = scene.getObject(0);
    TetraObject &obj2 = scene.getObject(1);

    obj2.translate(Real3(1.0, 0.0, 1.0) * 0.0707107);

    // ora vediamo la collisione 
    CollisionInfo coll_info = SAT_tet_tet(
                                    obj1, obj1.tetras[0], 
                                    obj2, obj2.tetras[0]);

    // stampiamo le info
    std::cout << "Intersecting: "      << (coll_info.intersecting ? "Yes" : "No") << "\n";
    std::cout << "Penetration depth: " << coll_info.penetration << "\n";
    std::cout << "Collision axis: ("   << coll_info.axis.x      << ", " 
                                       << coll_info.axis.y      << ", " 
                                       << coll_info.axis.z      << ")\n";
    std::cout << "Owner: "             << static_cast<int>(coll_info.owner) << "\n";

    while (!glfwWindowShouldClose(window)) {

        time = step * delta_t;

        loop_init();

        // XPBD_step(scene);

        rendering(); 

        loop_terminate();

        step++;
    }
}

int main() {

    if (!graphics_init()) return -1; 

    world_schema();

    graphics_close();

    return 0;
}
