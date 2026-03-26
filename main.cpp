#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <chrono>
#include <random>
#include <thread>
#include <string>
#include <sstream>
#include <unordered_map>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
namespace fs = std::filesystem;

#include "shader.cpp"
#include "scene.cpp"
#include "types.h"
#include "xpbd.cpp"
#include "ground.cpp"
#include "XMLparser.cpp"
#include "settings.cpp"
#include "settings.cpp"
#include "rendering.cpp"
#include "rigid.cpp"

#define MEASURE_TIME(function_call, accumulator)                                                   \
do {                                                                                               \
    auto start_##accumulator = std::chrono::high_resolution_clock::now();                          \
    function_call;                                                                                 \
    auto elapsed_##accumulator = std::chrono::high_resolution_clock::now() - start_##accumulator;  \
    accumulator += std::chrono::duration<Real, std::milli>(elapsed_##accumulator).count();         \
} while(0)

// Callback per ridimensionamento finestra
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

extern Real delta_t; 

extern unsigned int WIDTH;
extern unsigned int HEIGHT;
extern bool DO_VIDEO;
extern double targetFPS;

using string = std::string;

#define X(type, name, def_value) extern type name;
CONFIG_PARAMS
#undef X

extern Real3 gravity;

extern uint64_t frequency;

// const std::chrono::duration<double, std::milli> targetFrameDuration(1000.0 / targetFPS);
std::chrono::steady_clock::time_point frameStart;
std::chrono::steady_clock::time_point simulationStart;

GLFWwindow* window         = nullptr;
unsigned int objectProgram = 0;
unsigned int groundProgram = 0;
Scene  scene;
Ground ground;
SpringRenderer spring_renderer; 
FixedRigidSpringRenderer fixed_rigid_spring_renderer; 
RigidSpringRenderer rigid_spring_renderer; 

void parseArgument(int argc, char* argv[]) {

    if (argc < 2) return;

    try {
        wrap_compliance = std::stod(argv[1]);
    } 
    catch (const std::exception& e) {
        std::cerr << "Invalid double value: " << argv[1] << "\n";
        std::exit(1);
    }
}

void clear_folder(std::string folder_name, std::string extension) {
    fs::path folder(folder_name);

    fs::create_directories(folder);

    for (const auto& entry : fs::directory_iterator(folder)) {
        if (entry.is_regular_file() && entry.path().extension() == extension) {
            std::error_code ec; // per evitare eccezioni
            fs::remove(entry.path(), ec);
            if (ec) {
                std::cerr << "Errore cancellando " << entry.path() << ": " << ec.message() << "\n";
            }
        }
    }
}

void clear_video_folder() {
    clear_folder("..\\..\\video_frame", ".ppm");
}

bool graphics_init() 
{
    if (DO_VIDEO) clear_video_folder();

    if (!glfwInit()) 
    {
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

    // ========== ImGui Setup ==========
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();  // or ImGui::StyleColorsLight()
    
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    // =================================

    return init_shaders(objectProgram, groundProgram);
}

void graphics_close() 
{
    // ========== ImGui Cleanup ==========
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    // ===================================

    glDeleteProgram(objectProgram);
    glDeleteProgram(groundProgram);
    glfwTerminate();
}

// ############# LOOP #############

void loop_init() 
{
    frameStart = std::chrono::high_resolution_clock::now();
    
    glfwPollEvents();
    
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    ImGui::Begin("XPBD Controls");
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::End();
}

void loop_terminate(Real fps = 600.0) 
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);

    double targetMs = 1000.0 / fps;

    while (true) 
    {
        auto frameEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = frameEnd - frameStart;
        
        if (elapsed.count() >= targetMs) break;

        if (targetMs - elapsed.count() > 2.0) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// ############# RENDERING #############

glm::mat4 get_MVP(Real3 center) 
{    
    glm::mat4 model = glm::mat4(1.0f);
    // model = glm::rotate(model, (float)(0.1f * glfwGetTime()), glm::vec3(0.0, 1.0, 0.0));
    model = glm::rotate(model, glm::radians(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    glm::mat4 view = glm::lookAt(
        glm::vec3(center.x + camera_xoff, center.y + camera_yoff, camera_zoff),
        look_at_stack_center ? 
            glm::vec3(center.x, center.y, center.z) : 
            glm::vec3(center.x + camera_xoff, center.y + camera_yoff, center.z),
        glm::vec3(0.0f, 1.0f, 0.0f)
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

void set_shader(unsigned int shaderProgram, glm::mat4 &MVP) 
{
    glUseProgram(shaderProgram);
    int mvpLoc = glGetUniformLocation(shaderProgram, "MVP");
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, glm::value_ptr(MVP));
}

void background(float r, float g, float b) 
{
    glClearColor(r, g, b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void rendering(Real3 center = Real3(0.0)) 
{
    glm::mat4 MVP = get_MVP(center);

    set_shader(objectProgram, MVP);
    background(0.05f, 0.05f, 0.05f);
    for (RigidBox &obj : scene.rigid_objects) obj.draw();

    // for (TetraObject &obj : scene.objects)       obj.draw();
    // for (SceneObject &obj : scene.scene_objects) obj.draw();
    // for (Cloth &cloth : scene.cloths) cloth.draw();

    set_shader(groundProgram, MVP);
    // ground.draw();
    // ground.drawGrid();

    spring_renderer.draw(scene);
    rigid_spring_renderer.draw(scene);
    fixed_rigid_spring_renderer.draw(scene);
}

void null_rendering(Real3 center = Real3(0.0)) 
{
    glm::mat4 MVP = get_MVP(center);
    set_shader(objectProgram, MVP);
    background(0.1f, 0.3f, 0.2f);
}

// ######################################################################

void attach_vertices(Scene &scene, Index obj1, Index obj2, VertexIndex v1, VertexIndex v2, Real rest_length = 0.0) 
{
    TetraObject &o1 = scene.getObject(obj1);
    TetraObject &o2 = scene.getObject(obj2);

    SpringConstraint con(wrap_compliance, &o1, &o2, v1, v2, rest_length);
    scene.addConstraint(con);
}

std::string find_single_file(const std::string& folder_path) 
{
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            return entry.path().string(); // ritorna il path completo
        }
    }
    throw std::runtime_error("Nessun file trovato in " + folder_path);
}

std::vector<std::string> find_files_in_folder(const std::string& folder_path) 
{
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

struct Box 
{
    Real3 position;
    Real3 size;
    Real  weight;
};

std::pair<int, int> load_schema(const std::string& schema_path) 
{
    try 
    {
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

        
        std::pair<int, int> last_layer_idxs = {0,0};

        for (auto& layer_info : schema_XML["PalSchemaClass"][0]["LayerPaths"][0]["string"]) 
        {
            std::string layer_file_name = layer_info.value;

            for (auto& layer_XML : layer_XMLs) 
            {
                if (layer_XML.file_name != layer_file_name) continue;

                std::vector<XMLNode> boxes_position = layer_XML["LayerClass"][0]["SPDisposal"][0]["PalSchema_SPDisposalClass"];

                last_layer_idxs.first = boxes.size();

                for (auto& box_pos : boxes_position) 
                {
                    Real x = mult * box_pos.find_first("_x")->int_value;
                    Real z = mult * box_pos.find_first("_y")->int_value;
                    Real y = layer_num * height - 2.0;

                    if (box_pos.find_first("_rotation")->value == "true") 
                        boxes.push_back({Real3(x, y, z), Real3(width, height, length), weight});
                    else
                        boxes.push_back({Real3(x, y, z), Real3(length, height, width), weight});
                }

                last_layer_idxs.second = boxes.size();

                break;
            }
            layer_num++;
            // if (layer_num > 0) break;
        }
        
        for (const auto& box : boxes) 
        {
            scene.addObject(create_box(box.position, box.size.x, box.size.y, box.size.z));
        }

        return last_layer_idxs;
    } 
    catch (const std::exception& e) 
    {
        std::cerr << "Error loading schema: " << e.what() << "\n";
        throw;
    }
}

void wrap() 
{
    Real3 scene_min( std::numeric_limits<Real>::max());
    Real3 scene_max(-std::numeric_limits<Real>::max());

    for (const auto& obj : scene.objects) {
        scene_min = glm::min(scene_min, obj.aabb.min);
        scene_max = glm::max(scene_max, obj.aabb.max);
    }

    auto is_on_aabb_face = [&](const Real3& p, Real epsilon = 1e-5) -> bool 
    {
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

                    Real3 connection = obj1.positions[vi1] - obj2.positions[vi2];
                    Real distance    = glm::length(connection);
                    Real verticality = std::abs(glm::dot(glm::normalize(connection), Real3(0.0, 1.0, 0.0)));
                    if (distance < 0.4 && verticality < 0.8) {
                        if (is_on_aabb_face(obj1.positions[vi1]) && is_on_aabb_face(obj2.positions[vi2])) {
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

    spring_renderer.init(scene);

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

        Real3 velocity(8.0, 0.0, 0.0);
        if (time < 2.0)       velocity *= Real3(0.0, 0.0, 0.0) + time / 2.0;
        else if (time < 3.0)  velocity *= Real3(0.0, 0.0, 0.0) + (3.0 - time) / 1.0;
        else                  velocity *= 0.0;

        for (const auto& [key, init_pos] : positions) {
            TetraObject* obj      = key.first;
            VertexIndex vi        = key.second;
            positions[{obj, vi}] += velocity * delta_t;
            obj->positions[vi]    = positions[{obj, vi}];
        }

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

void cloth_world() {

    XPBD_init();

    uint64_t step = 0;
    Real time     = 0.0;

    scene.addCloth(Cloth(Real3(-2, 3, 0), Real3(2, 3, -4), 6, 1.0, 0.001));
    scene.cloths[0].fix_border(Border::TOP);

    clear_folder("..\\..\\animation", ".obj");

    while (!glfwWindowShouldClose(window)) {

        if (step % (4) == 0) {
            export_cloth_to_obj(scene, step / (4), 1.0);
        }

        time = step * delta_t;

        loop_init();

        XPBD_step(scene);

        rendering(); 

        loop_terminate();

        step++;
    }
}

void tetraball_world() {

    XPBD_init();

    uint64_t step = 0;
    Real time     = 0.0;

    // scene.addObject(load_tetrahedral_data("..\\..\\assets\\tetra_ball3.txt", 2.0));

    scene.addObject(create_parallelepiped_grid(Real3(-1.0, 1.0, -1.0), 3.0, 2.0, 2.0, 3, 2, 2));

    clear_folder("..\\..\\animation", ".obj");

    while (!glfwWindowShouldClose(window)) {

        if (step % 10 == 0) {
            export_tetra_surface_with_normals(scene, step / 10, 5, 2, 2, 1.0);
        }

        time = step * delta_t;

        loop_init();

        XPBD_step(scene);

        rendering(); 

        loop_terminate();

        step++;
    }
}

void collision_world() {
    XPBD_init();

    RigidBox b1(Real3(0.0, 0.0, 0.0), Real3(1.0, 1.0, 1.0), 1.0);
    RigidBox b2(Real3(0.9, 0.0, 0.0), Real3(1.0, 1.0, 1.0), 1.0);
    
    b2.rotate(Quat(0.1, 0.05, 0.02, 1.0));
    
    b1.update_world_vertices();
    b2.update_world_vertices();
    
    RigidCollisionInfo info = SAT_box_box(b1, b2);
    
    print_collision_info(info);
    
    export_collision_scene_to_obj(b1, b2, info, "..\\..\\animation\\collision_scene.obj");

}

void falling_rigid_world() {
    XPBD_init();

    uint64_t step = 0;
    Real time = 0.0;

    Real3 box_size = Real3(1.0, 1.0, 1.0);
    Real3 initial_position = Real3(3.0, 2.0, 0.0); 
    Real box_mass = 1.0;
    
    scene.addRigidObject(RigidBox(initial_position, box_size, box_mass));
    RigidBox& falling_box = scene.getRigidObject(0);

    Real3 world_anchor_point = Real3(0.0, 3.0, 0.0); 

    VertexIndex attached_vertex_index = 6; 
    Real3 local_attach_point = falling_box.body_vertices[attached_vertex_index];

    Real3 initial_world_vertex = falling_box.world_vertices[attached_vertex_index];
    Real rest_length = glm::length(initial_world_vertex - world_anchor_point);

    scene.addRigidConstraint(
        FixedRigidSpringConstraint(
            0.00001,                 // compliance del vincolo
            &falling_box,            // corpo rigido
            local_attach_point,      // punto di attacco locale sul corpo
            world_anchor_point,      // punto fisso nel mondo
            rest_length              // lunghezza a riposo
        )
    );

    fixed_rigid_spring_renderer.init(scene);

    clear_folder("..\\..\\animation", ".obj");

    while (!glfwWindowShouldClose(window)) {
        time = step * delta_t;

        loop_init();

        XPBD_step(scene);
        
        if (step % 10 == 0) {
            export_falling_rigid_to_obj(scene, step / 10, 1.0);
        }
        
        rendering();

        loop_terminate();

        step++;

        if (time > 10.0) break;
    }
}

void two_rigid_bodies_world() {
    XPBD_init();

    uint64_t step = 0;
    Real time = 0.0;

    Real3 box1_size = Real3(1.0, 1.0, 1.0);
    Real3 box1_initial_position = Real3(-3.0, 0.0, 0.0); 
    Real box1_mass = 1.0;
    
    scene.addRigidObject(RigidBox(box1_initial_position, box1_size, box1_mass));
    

    Real3 box2_size = Real3(0.8, 0.8, 0.8);
    Real3 box2_initial_position = Real3(3.0, 0.0, 0.0); 
    Real box2_mass = 1.0;
    
    scene.addRigidObject(RigidBox(box2_initial_position, box2_size, box2_mass));

    RigidBox& box1 = scene.getRigidObject(0);
    RigidBox& box2 = scene.getRigidObject(1);

    Real3 r1 = Real3( 0.5,  0.05, 0.0); // box1.body_vertices[6]; 
    Real3 r2 = Real3(-0.4,  0.05, 0.0); // box2.body_vertices[5];

    Real3 world_point1 = body_to_world(r1, box1.position, box1.orientation);
    Real3 world_point2 = body_to_world(r2, box2.position, box2.orientation);
    Real rest_length = glm::length(world_point1 - world_point2)/3.0;

    // Crea il vincolo tra i due corpi rigidi
    scene.addRigidConstraint(
        RigidSpringConstraint(
            0.005,                  // compliance del vincolo
            &scene.getRigidObject(0), // primo corpo rigido
            &scene.getRigidObject(1), // secondo corpo rigido  
            r1,                       // punto di attacco locale sul primo corpo
            r2,                       // punto di attacco locale sul secondo corpo
            rest_length               // lunghezza a riposo
        )
    );

    gravity = Real3(0.0, 0.0, 0.0);

    std::cout << "Constraint created \n";

    rigid_spring_renderer.init(scene);

    clear_folder("..\\..\\animation", ".obj");

    std::cout << "Inizio simulazione due corpi rigidi con vincolo a molla...\n";

    while (!glfwWindowShouldClose(window)) {
        time = step * delta_t;

        loop_init();

        XPBD_step(scene);
        
        // Esporta ogni 10 frame
        if (step % 10 == 0) {
            export_two_rigid_bodies_to_obj(scene, step / 10, 1.0);
        }
        
        rendering();

        loop_terminate();

        step++;


        if (time > 10.0) break;
    }
}

/*
void rigid_world_schema_stretch_stats(int num_iters)
{
    load_configuration_file("..\\..\\configurations\\c1.conf");

    Real scale_factor = 4.0;

    Box bpallet = load_rigid_schema("palleting_data\\" + schema_folder, scale_factor);

    AABB stack_aabb = scene.getRigidObject(0).aabb;
    for (Index i=0; i<scene.rigid_objects.size(); i++) {
        RigidBox &box = scene.getRigidObject(i);
        stack_aabb.expand(box.aabb.min);
        stack_aabb.expand(box.aabb.max);
    }

    Real3 p0(stack_aabb.min.x, stack_aabb.min.y, stack_aabb.min.z);
    Real3 p1(stack_aabb.min.x, stack_aabb.min.y, stack_aabb.max.z);
    Real3 p2(stack_aabb.max.x, stack_aabb.min.y, stack_aabb.max.z);
    Real3 p3(stack_aabb.max.x, stack_aabb.min.y, stack_aabb.min.z);

    Real3 p4(stack_aabb.min.x, stack_aabb.max.y, stack_aabb.min.z);
    Real3 p5(stack_aabb.min.x, stack_aabb.max.y, stack_aabb.max.z);
    Real3 p6(stack_aabb.max.x, stack_aabb.max.y, stack_aabb.max.z);
    Real3 p7(stack_aabb.max.x, stack_aabb.max.y, stack_aabb.min.z);

    constexpr int STAT_GRID_SIZE = 32;
    constexpr int SUB_FRAMES     = 3;
    constexpr int STAT_FRAMES    = 24*SUB_FRAMES*3;

    std::vector<Real> force_stats(STAT_FRAMES * 5 * STAT_GRID_SIZE * STAT_GRID_SIZE, 0.0);

    auto saveForceStats = [&](const char* filename) { 
        std::ofstream out(filename, std::ios::binary | std::ios::out | std::ios::trunc); 
        out.write(reinterpret_cast<const char*>(force_stats.data()), force_stats.size() * sizeof(Real)); 
        out.close(); 
    };

    Real length_x = stack_aabb.max.x - stack_aabb.min.x;
    Real length_y = stack_aabb.max.y - stack_aabb.min.y;
    Real length_z = stack_aabb.max.z - stack_aabb.min.z;

    Real slitta_height = 0.10 * scale_factor;
    Real pallet_height = 0.13 * scale_factor;

    auto whichBox = [&](Real3 p) -> Index {
        for (Index i=0; i<scene.rigid_objects.size()-1; i++) {
            if (scene.getRigidObject(i).aabb.contains(p, 1e-4)) return i;
        }
        return scene.rigid_objects.size()-1;
    };

    auto validBox = [&](Index i) -> bool {
        return i < scene.rigid_objects.size()-2;
    };

    auto attachPoint = [&](Real3 p, Index i) -> Real3 {
        RigidBox &box = scene.getRigidObject(i);
        return p - box.position;
    };

    auto wrapPlaneRandomLength = [&](Real3 p1, Real3 p2, Real length, uint8_t num_constraints, int plane_axis) {
        
        Real step_x = p2.x - p1.x;
        Real step_y = p2.y - p1.y;
        Real step_z = p2.z - p1.z;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<Real> dis(0.0, 1.0);
        
        for (uint8_t k = 0; k < num_constraints; k++) {

            Real3 pr1;
            Real3 pr2;

            while(true) { 
                Real u1 = dis(gen);
                Real v1 = dis(gen);
                Real w1 = dis(gen);
                pr1     = p1 + Real3(u1 * step_x, v1 * step_y, w1 * step_z);

                Real rand_direction = dis(gen) * 2.0 * glm::pi<Real>();
                Real cos_length = glm::cos(rand_direction) * length;
                Real sin_length = glm::sin(rand_direction) * length;
                switch(plane_axis) {
                    case 0: pr2 = pr1 + Real3(0.0, cos_length, sin_length); break;
                    case 1: pr2 = pr1 + Real3(cos_length, 0.0, sin_length); break;
                    case 2: pr2 = pr1 + Real3(cos_length, sin_length, 0.0); break;
                }
                bool inside = (pr2.x >= p1.x && pr2.x <= p2.x &&
                               pr2.y >= p1.y && pr2.y <= p2.y &&
                               pr2.z >= p1.z && pr2.z <= p2.z);

                if (!inside) continue;

                Index b1 = whichBox(pr1);
                Index b2 = whichBox(pr2);

                if (validBox(b1) && validBox(b2) && (b1 != b2)) break;
            }

            
            Index b1 = whichBox(pr1);
            Index b2 = whichBox(pr2);
            
            Real3 a1 = attachPoint(pr1, b1);
            Real3 a2 = attachPoint(pr2, b2);

            RigidBox *bx1  = validBox(b1)  ? &scene.getRigidObject(b1)  : nullptr;
            RigidBox *bx2  = validBox(b2)  ? &scene.getRigidObject(b2) : nullptr;

            if (validBox(b1) && validBox(b2) && (b1 != b2)) {
                Real distance = glm::length(pr1 - pr2);
                scene.addRigidConstraint(
                    RigidSpringConstraint(
                        wrap_compliance, 
                        bx1, bx2, 
                        a1, a2, 
                        distance));
            }
        }
    };

    auto attachBase = [&](Real3 p1, Real3 p2) {
        for (Index bi=0; bi<scene.rigid_objects.size()-1; bi++) {
            RigidBox &box = scene.getRigidObject(bi);
            for (int vi=0; vi<8; vi++) {
                Real3 v = box.world_vertices[vi];

                if ( (v.x >= std::min(p1.x, p2.x) - 1e-4) && (v.x <= std::max(p1.x, p2.x) + 1e-4) &&
                     (v.z >= std::min(p1.z, p2.z) - 1e-4) && (v.z <= std::max(p1.z, p2.z) + 1e-4) &&
                     (std::abs(v.y - p1.y) < 1e-4) ) {
                    
                    scene.addRigidConstraint(
                            FixedRigidSpringConstraint(
                                base_attach_compliance, 
                                &box, 
                                box.body_vertices[vi], 
                                v,
                                0.0));
                }
            }   
        }
    };

    for (int it = 0; it < num_iters; it++) 
    {
        std::cout << "=== STATISTICS ITERATION " << it << " ===" << std::endl;

        scene.clear();
                
        Box bpallet = load_rigid_schema("palleting_data\\" + schema_folder, scale_factor);

        XPBD_init();

        uint64_t step = 0;
        Real time     = 0.0;

        Real3 center = (stack_aabb.min + stack_aabb.max) * 0.5;

        // PALLET HITBOX
        scene.addRigidObject(
                RigidBox(
                    Real3(center.x, -2.0 - bpallet.size.y/2.0 - 0.001, center.z),
                    Real3(bpallet.size.x + 1.0, bpallet.size.y, bpallet.size.z + 1.0),
                    1.0));
        scene.getRigidObject(scene.rigid_objects.size()-1).make_static();

        // GROUND
        scene.addRigidObject(
                RigidBox(
                    Real3(28.5, - 3.0 - pallet_height - slitta_height,  center.z),
                    Real3(62.0, 2.0,  bpallet.size.z + 4.0),
                    1.0));
        scene.getRigidObject(scene.rigid_objects.size()-1).make_static();

        std::vector<std::array<Index, 3>> constr_to_grid_index;
        Real x_step = length_x / (Real)(STAT_GRID_SIZE);
        Real y_step = length_y / (Real)(STAT_GRID_SIZE);
        Real z_step = length_z / (Real)(STAT_GRID_SIZE);

        wrapPlaneRandomLength(p1, p6, glm::max(length_x, length_y) * glm::clamp(wrap_param, 0.1, 0.9), wrap_steps, 2);
        wrapPlaneRandomLength(p0, p7, glm::max(length_x, length_y) * glm::clamp(wrap_param, 0.1, 0.9), wrap_steps, 2);
        wrapPlaneRandomLength(p0, p5, glm::max(length_z, length_y) * glm::clamp(wrap_param, 0.1, 0.9), wrap_steps, 0);
        wrapPlaneRandomLength(p3, p6, glm::max(length_z, length_y) * glm::clamp(wrap_param, 0.1, 0.9), wrap_steps, 0);
        wrapPlaneRandomLength(p4, p6, glm::max(length_x, length_z) * glm::clamp(wrap_param, 0.1, 0.9), wrap_steps, 1); 

        for (int c=0; c<scene.rigid_constraints.size(); c++) 
        {
            Index plane_index = c / wrap_steps;

            auto& constraint = scene.rigid_constraints[c];

            RigidBox *b1 = constraint.b1;
            RigidBox *b2 = constraint.b2;

            Real3 r1 = constraint.r1;
            Real3 r2 = constraint.r2;

            Real3 w1 = body_to_world(r1, b1->position, b1->orientation);
            Real3 w2 = body_to_world(r2, b2->position, b2->orientation);

            Real3 pbl = Real3(0.0);

            switch(plane_index) 
            {
                case 0: pbl = p1; break; // z-front
                case 1: pbl = p0; break; // z-back
                case 2: pbl = p0; break; // x-left
                case 3: pbl = p3; break; // x-right
                case 4: pbl = p4; break; // y-up
            }

            w1       = w1 - pbl;
            w2       = w2 - pbl;
            Real3 w3 = (w1 + w2) * 0.5 - pbl;

            Index x1i = static_cast<Index>(floor(w1.x / x_step));
            Index y1i = static_cast<Index>(floor(w1.y / y_step));
            Index z1i = static_cast<Index>(floor(w1.z / z_step));

            Index x2i = static_cast<Index>(floor(w2.x / x_step));
            Index y2i = static_cast<Index>(floor(w2.y / y_step));
            Index z2i = static_cast<Index>(floor(w2.z / z_step));

            Index x3i = static_cast<Index>(floor(w3.x / x_step));
            Index y3i = static_cast<Index>(floor(w3.y / y_step));
            Index z3i = static_cast<Index>(floor(w3.z / z_step));

            Index stat1_index;
            Index stat2_index;
            Index stat3_index;

            if (plane_index == 0 || plane_index == 1) 
            {
                stat1_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + y1i * STAT_GRID_SIZE + x1i;
                stat2_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + y2i * STAT_GRID_SIZE + x2i;
                stat3_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + y3i * STAT_GRID_SIZE + x3i;
            }
            else if (plane_index == 2 || plane_index == 3) 
            {
                stat1_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + y1i * STAT_GRID_SIZE + z1i;
                stat2_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + y2i * STAT_GRID_SIZE + z2i;
                stat3_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + y3i * STAT_GRID_SIZE + z3i;
            }
            else if (plane_index == 4) 
            {
                stat1_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + z1i * STAT_GRID_SIZE + x1i;
                stat2_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + z2i * STAT_GRID_SIZE + x2i;
                stat3_index = plane_index * STAT_GRID_SIZE * STAT_GRID_SIZE + z3i * STAT_GRID_SIZE + x3i;
            }

            constr_to_grid_index.push_back({stat1_index, stat2_index, stat3_index});
        }

        attachBase(p0, p2);

        RigidBox &pallet_hitbox = scene.rigid_objects[scene.rigid_objects.size()-2];

        center.x = 0.0;

        Real3 vel_vector(0.0, 0.0, 0.0); 
        Real3 acc_vector(0.0, 0.0, 0.0);

        while (!glfwWindowShouldClose(window)) {

            if (step % 1000 == 0) std::cout << "Step: " << step << ", Time: " << time << " s\r";

            time = step * delta_t;

            if      (time < acc_time)                          acc_vector = Real3(acceleration, 0.0, 0.0);
            else if (time < acc_time + dec_time)               acc_vector = Real3(deceleration, 0.0, 0.0);
            else if (time <= acc_time + dec_time + still_time) acc_vector = Real3(0.0, 0.0, 0.0);
            else break;

            vel_vector   += acc_vector * delta_t;
            Real3 offset  = vel_vector * delta_t;
            for (FixedRigidSpringConstraint &c : scene.fixed_rigid_constraints)
                c.world_attach += offset;
            center += offset;
            pallet_hitbox.translate(offset);

            XPBD_step(scene);

            if (step % (25*10/SUB_FRAMES) == 0) {
                int frame = step / (25*10/SUB_FRAMES);
                if (frame >= STAT_FRAMES) break;
                if (frame < STAT_FRAMES) {
                    for (int c=0; c<scene.rigid_constraints.size(); c++) {
                        auto& constraint = scene.rigid_constraints[c];
                        auto& grid_indices = constr_to_grid_index[c];

                        Real force_magnitude = constraint.lambda / (delta_t*delta_t);
                        Real3 w1 = body_to_world(constraint.r1, constraint.b1->position, constraint.b1->orientation);
                        Real3 w2 = body_to_world(constraint.r2, constraint.b2->position, constraint.b2->orientation);
                        // Real3 d  = glm::normalize(w1 -  w2);
                        Real stretching = glm::length(w1 - w2) - constraint.rest_length;

                        // if (frame * (5 * STAT_GRID_SIZE * STAT_GRID_SIZE) + grid_indices[0] >= STAT_FRAMES * 5 * STAT_GRID_SIZE * STAT_GRID_SIZE ||
                        //     frame * (5 * STAT_GRID_SIZE * STAT_GRID_SIZE) + grid_indices[1] >= STAT_FRAMES * 5 * STAT_GRID_SIZE * STAT_GRID_SIZE) {
                        //     continue;
                        // }

                        force_stats[frame * (5 * STAT_GRID_SIZE * STAT_GRID_SIZE) + grid_indices[0]] += glm::abs(stretching);
                        force_stats[frame * (5 * STAT_GRID_SIZE * STAT_GRID_SIZE) + grid_indices[1]] += glm::abs(stretching);
                        force_stats[frame * (5 * STAT_GRID_SIZE * STAT_GRID_SIZE) + grid_indices[2]] += glm::abs(stretching);
                    }
                }
            }

            step++;
        }
        std::cout << '\n';
    }

    std::string filepath = "..\\..\\statistics\\rigid_schema_stretch_stats_" + schema_folder + ".bin";
    saveForceStats(filepath.c_str());
}
*/

// *******************************
// **         RIGID             **
// *******************************

enum class AppState
{
    SETUP,   // Configure parameters
    RUNNING, // Simulation running
    FINISHED // Simulation complete
};

AppState app_state    = AppState::SETUP;
bool start_simulation = false;
bool reset_simulation = false;
bool end_simulation   = false;

template <typename T>
T snap(T &v, T step) { if (step <= 0) return v;  return v = std::round(v / step) * step; }

#define SliderReal(description, param, min, max) ImGui::SliderScalar(description, ImGuiDataType_Double, param, min, max, "%.2f")

struct DataCollection
{
    static constexpr size_t DataPointsPerSecond = 50;

    struct Flags 
    {
        bool times            = true;
        bool accelerations    = true;
        bool displacements    = true;
        bool angles           = true;
        bool elastic_energies = true;
        bool max_force        = true;
        bool total_force      = true;
        bool total_stretch    = true;
        bool kinetic_energy   = true;
        bool com_drift        = true;
    } print_flags;

    std::string postfix = "";

    std::vector<Real> displacements;
    std::vector<Real> times;
    std::vector<Real> angles;
    std::vector<Real> accelerations;
    std::vector<Real> elastic_energies;
    std::vector<Real> max_force_recorded;
    std::vector<Real> total_force_recorded;
    std::vector<Real> total_stretch_x;
    std::vector<Real> total_stretch_y;
    std::vector<Real> total_stretch_z;
    std::vector<Real> kinetic_energy;
    std::vector<Real3> com_drift;

    Real3 initial_com;
    int last_layer_idxs[2];

    DataCollection() {}

    void init(Scene& scene, const AABB& stack_aabb, int last_layer_indexes[2], size_t num_data_points)
    {
        displacements.clear();
        angles.clear();
        accelerations.clear();
        elastic_energies.clear();
        max_force_recorded.clear();
        total_force_recorded.clear();
        total_stretch_x.clear();
        total_stretch_y.clear();
        total_stretch_z.clear();
        kinetic_energy.clear();
        com_drift.clear();
        times.clear();

        displacements.reserve(num_data_points);
        angles.reserve(num_data_points);
        accelerations.reserve(num_data_points);
        elastic_energies.reserve(num_data_points);
        max_force_recorded.reserve(num_data_points);
        total_force_recorded.reserve(num_data_points);
        total_stretch_x.reserve(num_data_points);
        total_stretch_y.reserve(num_data_points);
        total_stretch_z.reserve(num_data_points);
        kinetic_energy.reserve(num_data_points);
        com_drift.reserve(num_data_points);
        times.reserve(num_data_points);

        last_layer_idxs[0] = last_layer_indexes[0];
        last_layer_idxs[1] = last_layer_indexes[1];

        Real3 current_com_sum = Real3(0.0);
        Real total_mass       = 0.0;

        for(size_t i=0; i<scene.rigid_objects.size()-2; i++) 
        {
            const auto& box = scene.getRigidObject(i);

            current_com_sum += box.position * box.mass;   
            total_mass      += box.mass;
        }

        initial_com = current_com_sum / total_mass;
    }
    
    void update(Scene& scene, Real time, Real3 center, Real base_x, Real base_y, Real3 acc_vector)
    {
        Real x = std::numeric_limits<Real>::max();
        Real y = 0.0;

        for (int i=last_layer_idxs[0]; i<last_layer_idxs[1]; i++) 
        {
            const RigidBox &box = scene.rigid_objects[i];
            if (box.min_x() < x) 
            {
                x = box.min_x();
                y = box.max_y();
            }
        }

        Real disp  = x - base_x;
        Real angle = glm::degrees(atan2(y - base_y, disp)) - 90.0;

        displacements.push_back(disp);
        angles.push_back(angle);
        accelerations.push_back(acc_vector.x);
        times.push_back(time);

        // ===================================================================

        Real total_elastic_energy = 0.0;
        Real max_force            = std::numeric_limits<Real>::lowest();
        Real total_magnitude      = 0.0;
        Real3 total_stretch       = Real3(0.0);

        for (const auto& constraint : scene.rigid_constraints) 
        {   
            const RigidBox *b1 = constraint.b1;
            const RigidBox *b2 = constraint.b2;

            if (b1 == b2) continue;

            Real3 p1 = body_to_world(constraint.r1, b1->position, b1->orientation);
            Real3 p2 = body_to_world(constraint.r2, b2->position, b2->orientation);

            Real3 dir = p2 - p1;
            Real dist = glm::length(dir);
            Real C    = dist - constraint.rest_length;

            if (C <= 0.0) continue;

            Real3 stretch_dir = dir / dist;
            Real3 stretch_vec = glm::abs(stretch_dir) * C;

            Real force_scalar = constraint.lambda / (delta_t * delta_t);

            total_stretch += stretch_vec;

            total_magnitude += std::abs(force_scalar);

            max_force = std::max(max_force, std::abs(force_scalar));

            if (constraint.compliance > 0.0) total_elastic_energy += (C * C) / (2.0 * constraint.compliance);
        }
        
        elastic_energies.push_back(total_elastic_energy);
        max_force_recorded.push_back(max_force);
        total_force_recorded.push_back(total_magnitude);
        total_stretch_x.push_back(total_stretch.x);
        total_stretch_y.push_back(total_stretch.y);
        total_stretch_z.push_back(total_stretch.z);

        // ==================================================================

        Real total_ke = 0.0;

        Real3 current_com_sum = Real3(0.0);
        Real total_mass       = 0.0;

        for(size_t i=0; i<scene.rigid_objects.size()-2; i++) 
        {
            const auto& box = scene.getRigidObject(i);

            Real3x3 R       = quat_to_rotmat(box.orientation);
            Real3x3 I_world = R * box.inertia_tensor * glm::transpose(R);

            total_ke += 0.5 * box.mass * glm::dot(box.velocity, box.velocity);
            total_ke += 0.5 * glm::dot(box.angular_velocity, I_world * box.angular_velocity);

            current_com_sum += box.position * box.mass;   
            total_mass      += box.mass;
        }

        Real3 current_com = current_com_sum / total_mass;
        
        current_com.x -= center.x;

        kinetic_energy.push_back(total_ke);
        com_drift.push_back(current_com - initial_com);
    }

    void print()
    {
        static auto pythonListPrint = [&](std::string list_name, const std::vector<Real>& vec) 
        {
            std::cout << list_name << (postfix != "" ? "_" : "") << postfix << " = [";
            for (size_t i=0; i<vec.size(); i++) 
            {
                std::cout << vec[i];
                if (i < vec.size()-1) std::cout << ", ";
            }
            std::cout << "]\n\n";
        };

        static auto pythonReal3Print = [&](std::string list_name, const std::vector<Real3>& vec, int axis) 
        {
            std::cout << list_name << (postfix != "" ? "_" : "") << postfix << " = [";
            for (size_t i=0; i<vec.size(); i++) 
            {
                std::cout << vec[i][axis]; 
                if (i < vec.size()-1) std::cout << ", ";
            }
            std::cout << "]\n\n";
        };

        std::cout << "# --- Data Export Start ---\n\n";

        if (print_flags.times)            pythonListPrint("times", times);
        if (print_flags.accelerations)    pythonListPrint("accelerations", accelerations);
        if (print_flags.displacements)    pythonListPrint("displacements", displacements);
        if (print_flags.angles)           pythonListPrint("angles", angles);
        if (print_flags.elastic_energies) pythonListPrint("elastic_energies", elastic_energies);
        if (print_flags.max_force)        pythonListPrint("max_force_recorded", max_force_recorded);
        if (print_flags.total_force)      pythonListPrint("total_force_recorded", total_force_recorded);
        if (print_flags.kinetic_energy)   pythonListPrint("kinetic_energy", kinetic_energy);

        if (print_flags.total_stretch) 
        {
            pythonListPrint("total_stretch_x", total_stretch_x);
            pythonListPrint("total_stretch_y", total_stretch_y);
            pythonListPrint("total_stretch_z", total_stretch_z);
        }

        if (print_flags.com_drift) {
            pythonReal3Print("com_drift_x", com_drift, 0);
            pythonReal3Print("com_drift_y", com_drift, 1);
            pythonReal3Print("com_drift_z", com_drift, 2);
        }

        std::cout << "# --- Data Export End ---\n" << std::endl;
    }

    void render_ui()
    {
        ImGui::Begin("Data Collection", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        if (ImGui::Checkbox("Enable Data Collection", &collect_data)) 
        { 
            reset_simulation = true; 
        }

        if (collect_data)
        {
            ImGui::Separator();
            ImGui::Text("Selection for Python Export:");
            
            ImGui::Checkbox("Times",                  &print_flags.times);
            ImGui::Checkbox("Accelerations",          &print_flags.accelerations);
            ImGui::Checkbox("Displacements",          &print_flags.displacements);
            ImGui::Checkbox("Tilt Angles",            &print_flags.angles);
            ImGui::Checkbox("Elastic Energies",       &print_flags.elastic_energies);
            ImGui::Checkbox("Max Forces",             &print_flags.max_force);
            ImGui::Checkbox("Total Forces",           &print_flags.total_force);
            ImGui::Checkbox("Material Stretch (XYZ)", &print_flags.total_stretch);
            ImGui::Checkbox("Kinetic Energy",         &print_flags.kinetic_energy);
            ImGui::Checkbox("CoM Drift (XYZ)",        &print_flags.com_drift);

            char buf[128];
            strncpy(buf, postfix.c_str(), sizeof(buf));
            if (ImGui::InputText("Postfix", buf, sizeof(buf))) { postfix = std::string(buf);  }

            ImGui::Separator();

            if(ImGui::Button("Clear Output")) std::system("cls");

            static auto setAll = [&](bool value)
            {
                print_flags.times            = value;
                print_flags.accelerations    = value;
                print_flags.displacements    = value;
                print_flags.angles           = value;
                print_flags.elastic_energies = value;
                print_flags.max_force        = value;
                print_flags.total_force      = value;
                print_flags.total_stretch    = value;
                print_flags.kinetic_energy   = value;
                print_flags.com_drift        = value;
            };

            if(ImGui::Button("Set All False")) setAll(false);
            if(ImGui::Button("Set All True"))  setAll(true);
        }

        ImGui::End();
    }
};

static DataCollection data;


void render_ui(uint64_t steps, Real time, Real total_physics_time) 
{
    if (app_state == AppState::SETUP)
    {
        ImGui::Begin("Simulation Setup", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        static constexpr Real possible_values[] = {
            0.000001, 0.0000025, 0.000005, 0.00001, 
            0.000025, 0.00005, 0.0001, 0.00025, 0.0005
        };

        const int num_vals = IM_ARRAYSIZE(possible_values);

        auto DiscreteSlider = [&](const char* label, Real& current_val) 
        {    
            int current_idx = 0;
            for (int i = 0; i < num_vals; i++) 
            {
                if (std::abs(current_val - possible_values[i]) < 1e-12) 
                { 
                    current_idx = i; 
                    break; 
                }
            }

            if (ImGui::SliderInt(label, &current_idx, 0, num_vals - 1, "")) 
            {
                reset_simulation = true;
                current_val = possible_values[current_idx];
            }
            
            ImGui::SameLine();
            ImGui::Text("%.7f", current_val);
        };

        ImGui::Separator();
        ImGui::Text("XPBD Settings");

        if(ImGui::SliderInt("Sim. Heartz", &xpbd_steps_x_second, 100, 3000)) 
        { 
            xpbd_steps_x_second = ((xpbd_steps_x_second + 25) / 50) * 50;
            reset_simulation    = true;  
        }

        if(ImGui::SliderInt("Constr. Iterations", &xpbd_iters_x_step, 1, 50)) { reset_simulation = true; }

        ImGui::Separator();
        ImGui::Text("Compliance Settings");

        DiscreteSlider("Collision##1",    coll_compliance);
        DiscreteSlider("Wrap##1",         wrap_compliance);
        DiscreteSlider("Base Attach##1",  base_attach_compliance);

        ImGui::Separator();
        ImGui::Text("Motion Profile");

        static float accel_magnitude = 10.0f; 

        if (ImGui::SliderFloat("Acceleration", &accel_magnitude, 0.0f, 20.0f)) 
        {
            snap(accel_magnitude, 0.2f);
            acceleration = accel_magnitude;
            deceleration = - (acc_time * accel_magnitude) / dec_time;
            reset_simulation = true;
        }

        Real min = 0.1;
        Real max = 5.0;

        if (SliderReal("Acceleration Time", &acc_time, &min, &max)) 
        {
            snap(acc_time, 0.1);
            deceleration = - (acc_time * accel_magnitude) / dec_time;
            reset_simulation = true;
        }

        if (SliderReal("Deceleration Time", &dec_time, &min, &max)) 
        {
            snap(dec_time, 0.1);
            deceleration = - (acc_time * accel_magnitude) / dec_time;
            reset_simulation = true;
        }

        if (SliderReal("Still Time", &still_time, &min, &max)) 
        {
            snap(still_time, 0.1);
            reset_simulation = true;
        }

        ImGui::Separator();

        Real min_mu = 0.0;
        Real max_mu = 1.0;

        SliderReal("Mu Dynamic", &mu_dynamic, &min_mu, &max_mu);

        ImGui::Separator();

        static std::vector<std::string> schema_options;
        static int current_schema = 0;
        static bool initialized = false;

        if (!initialized) 
        {
            schema_options.clear();
            std::string path = "..\\..\\palleting_data";

            try
            {
                if (fs::exists(path) && fs::is_directory(path)) 
                {
                    for (const auto& entry : fs::directory_iterator(path)) 
                    {
                        if (entry.is_directory()) 
                        {
                            std::string schema_name = entry.path().filename().string();

                            if (schema_name.find("_rot") == std::string::npos) schema_options.push_back(schema_name);
                        }
                    }
                }
            }
            catch (const fs::filesystem_error& e) { std::cerr << "Error scanning directory: " << e.what() << "\n"; }

            if (schema_options.empty()) schema_options.push_back("default_schema");

            auto it = std::find(schema_options.begin(), schema_options.end(), schema_folder);
            if (it != schema_options.end()) 
            {
                current_schema = std::distance(schema_options.begin(), it);
            } 
            else 
            {
                current_schema = 0;
                schema_folder  = schema_options[0];
            }
            
            initialized = true;
        }

        if (!schema_options.empty() && ImGui::BeginCombo("Schema Folder", schema_options[current_schema].c_str())) 
        {
            for (int i = 0; i < (int)schema_options.size(); i++) 
            {
                bool is_selected = (current_schema == i);
                if (ImGui::Selectable(schema_options[i].c_str(), is_selected)) 
                {
                    current_schema   = i;
                    schema_folder    = schema_options[i];
                    reset_simulation = true;
                }
                if (is_selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        struct WrapLimits {
            const char* param_label;
            int min_s, max_s;
            double min_p, max_p;
        };

        static constexpr WrapLimits wrap_limits[] = {
            // Label          MinS  MaxS   MinP   MaxP
            { "None",         0,    0,     0.0,   0.0   }, // none
            { "None",         1,    100,   0.0,   10.0  }, // grid
            { "Offset",       1,    100,   0.0,   2.0   }, // shifted
            { "Angle",        1,    100,   0.0,   90.0  }, // rotated
            { "Angle",        1,    100,   0.0,   90.0  }, // rotated_mirrored
            { "Length",       1,    500,   0.01,  1.0   }, // random_length
            { "None",         1,    500,   0.0,   50.0  }, // random
            { "Margin",       1,    40,    0.0,   2.0   }  // edges
        };

        static constexpr char* options[] = {
            #define X(name, string_val) #string_val, 
            WRAP_MODES
            #undef X
        };

        static_assert(IM_ARRAYSIZE(wrap_limits) == IM_ARRAYSIZE(options), 
            "WrapLimits table size does not match the number of WRAP_MODES defined in X-Macros!");

        auto render_wrap_config = [](const char* label, std::string& type, int& steps, Real& param) 
        {
            const int num_options = IM_ARRAYSIZE(options);
            int current = 0;

            for (int i = 0; i < num_options; i++) if (type == options[i]) { current = i; break; }

            ImGui::Text("%s", label);
            if (ImGui::Combo(("Type##" + std::string(label)).c_str(), &current, options, num_options)) 
            {
                const auto& new_limits = wrap_limits[current];

                steps = new_limits.min_s + (new_limits.max_s - new_limits.min_s) / 10;
                param = new_limits.min_p + (new_limits.max_p - new_limits.min_p) * 0.1;
                type  = options[current];
                reset_simulation = true;
            }

            const auto& limits = wrap_limits[current];

            if (current != 0)
            {
                if (ImGui::SliderInt(("Steps##" + std::string(label)).c_str(), &steps, limits.min_s, limits.max_s)) 
                    reset_simulation = true; 
                
                if (string(limits.param_label) != "None")
                {
                    if (ImGui::SliderScalar((
                            std::string(limits.param_label) + "##" + label).c_str(), ImGuiDataType_Double, 
                            &param, &limits.min_p, &limits.max_p, "%.2f"))
                    {
                        reset_simulation = true;
                    }
                }
            }
        };

        ImGui::Separator();
        render_wrap_config("Primary Wrap", wrap_type, wrap_steps, wrap_param);

        ImGui::Separator();
        render_wrap_config("Secondary Wrap", wrap_type_sec, wrap_steps_sec, wrap_param_sec);

        ImGui::Separator();
        ImGui::Text("Export & Physics");

        if (ImGui::Checkbox("Export OBJ", &export_obj)) { }

        static int vfps = 24, slowing = 3;
        if (export_obj)
        {
            char buf[128];
            strncpy(buf, prefix.c_str(), sizeof(buf));
            if (ImGui::InputText("File Prefix", buf, sizeof(buf))) { prefix = std::string(buf);  }

            if(ImGui::SliderInt("Video FPS", &vfps, 1, 24))  { reset_simulation = true; video_fps = vfps*slowing; }
            if(ImGui::SliderInt("Slowing", &slowing, 1, 10)) { reset_simulation = true; video_fps = vfps*slowing; }

            ImGui::Checkbox("Export Stretch Perc as Color", &export_stretch_perc);
        }

        ImGui::Separator();

        ImGui::Checkbox("Render Tearing", &render_tearing);

        if (ImGui::Checkbox("Apply Tearing", &apply_tearing)) { }

        Real min_stretch = 0.0, max_stretch = 0.2; 
        
        if (apply_tearing || render_tearing)
        {
            ImGui::Indent(); 
            ImGui::SliderScalar("Stretch Limit", ImGuiDataType_Double, 
                                &tearing_stretch_percentage, 
                                &min_stretch, &max_stretch, "%.3f");
            ImGui::Unindent();
        }

        // if (apply_tearing)
        // {
        //     // Real min_force = 0.0, max_force = 1000.0; 
        //     // ImGui::SliderScalar("Force Limit", 
        //     //                     ImGuiDataType_Double, 
        //     //                     &force_tearing_threshold, 
        //     //                     &min_force, &max_force, "%.3f");
        // }

        ImGui::Separator();
        if (ImGui::Button("Clear animation folder")) clear_folder("..\\..\\animation", ".obj");

        ImGui::Separator();
        if (ImGui::Button("Export Starting Configuration"))
        {
            export_scene_to_obj(scene, 0, scale_factor, Real3(0.0f), prefix);
            export_wrap_to_obj(scene,  0, scale_factor, Real3(0.0f), prefix);
        }

        ImGui::Separator();
        if (ImGui::Button("Start Simulation", ImVec2(200, 40))) 
        {
            start_simulation = true;
            app_state        = AppState::RUNNING;
        }
        
        ImGui::End();

        ImGui::Begin("Camera Setup", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        Real cam_min = -5.0; Real cam_max = 5.0;
        Real cam_min_z = -10.0; Real cam_max_z = 10.0;

        SliderReal("Camera X offset", &camera_xoff, &cam_min, &cam_max);
        SliderReal("Camera Y offset", &camera_yoff, &cam_min, &cam_max);
        SliderReal("Camera Z offset", &camera_zoff, &cam_min_z, &cam_max_z);

        if (ImGui::Checkbox("LookAt StckCenter", &look_at_stack_center)) 
        {
            camera_xoff = 0.0;
            camera_yoff = 0.0;
        }

        ImGui::End();

        data.render_ui();
    }
    else if (app_state == AppState::RUNNING) 
    {
        ImGui::Begin("Simulation Running");

        ImGui::Text("Step: %d", steps);
        ImGui::Text("Time: %.2f s", time);
        // ImGui::Text("Velocity: %.2f m/s", glm::length(vel_vector));
        ImGui::Text("Physics Time: %.2f ms", (total_physics_time / steps));

        ImGui::Separator();
        if (ImGui::Button("Stop Simulation")) app_state = AppState::FINISHED;
        ImGui::End();
    }
    else if (app_state == AppState::FINISHED) 
    {
        ImGui::Begin("Simulation Complete");

        ImGui::Text("Total Steps: %d",           steps);
        ImGui::Text("Total Time: %.2f s",        time);
        ImGui::Text("Avg Physics Time: %.2f ms", (total_physics_time / steps));
        ImGui::Text("Total XPBD Time: %.4f ms",  total_physics_time);
        ImGui::Separator();

        if (ImGui::Button("New Simulation", ImVec2(200, 40))) 
        {
            reset_simulation = true;
            app_state        = AppState::SETUP;
        }
        ImGui::End();
    }
}

Box load_rigid_schema(const std::string& schema_path, Real scale, int *last_layer_idxs = nullptr) 
{
    try 
    {
        XMLParser pallet_parser(find_single_file("..\\..\\"    + schema_path + "\\Pallet"));
        XMLParser secondary_parser(find_single_file("..\\..\\" + schema_path + "\\SecondaryPackaging"));
        XMLParser schema_parser(find_single_file("..\\..\\"    + schema_path + "\\PalletisingSchema"));

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

        Real weight =         mult * secondary_XML.find_first("Weight")->int_value;
        Real height = scale * mult * secondary_XML.find_first("Height")->int_value;
        Real width  = scale * mult * secondary_XML.find_first("Width")->int_value;
        Real length = scale * mult * secondary_XML.find_first("Length")->int_value;

        Real total_weight = 0.0;

        std::vector<Box> boxes;

        int layer_num = 0;
        for (auto& layer_info : schema_XML["PalSchemaClass"][0]["LayerPaths"][0]["string"]) 
        {
            std::string layer_file_name = layer_info.value;

            for (auto& layer_XML : layer_XMLs) 
            {
                if (layer_XML.file_name != layer_file_name) continue;

                std::vector<XMLNode> boxes_position = layer_XML["LayerClass"][0]["SPDisposal"][0]["PalSchema_SPDisposalClass"];

                if (last_layer_idxs != nullptr) last_layer_idxs[0] = (int) boxes.size();

                for (auto& box_pos : boxes_position) 
                {
                    Real x = scale * mult * box_pos.find_first("_x")->int_value;
                    Real z = scale * mult * box_pos.find_first("_y")->int_value;
                    Real y = layer_num * height - 2.0;

                    if (box_pos.find_first("_rotation")->value == "true") 
                        boxes.push_back({Real3(x, y, z), Real3(width, height, length), weight});
                    else
                        boxes.push_back({Real3(x, y, z), Real3(length, height, width), weight});

                    total_weight += weight;
                }

                if (last_layer_idxs != nullptr) last_layer_idxs[1] = (int) boxes.size();

                break;
            }
            layer_num++;
        }

        // std::cout << "Total boxes weight: " << total_weight << " kg\n";

        for (const auto& box : boxes) 
        {
            scene.addRigidObject(RigidBox(box.position + box.size*0.5, box.size, box.weight));
        }

        // PALLET

        XMLNode pallet = pallet_XML["PalletClass"][0];
        Real size_x = scale * mult * pallet.find_first("DimX")->int_value;
        Real size_z = scale * mult * pallet.find_first("DimY")->int_value;
        Real size_y = scale * mult * pallet.find_first("Dim_Z")->int_value;

        return {Real3(0.0), Real3(size_x, size_y, size_z), 1.0};
    } 
    catch (const std::exception& e) { std::cerr << "Error loading schema: " << e.what() << "\n"; }

    return {Real3(0.0), Real3(1.0), 1.0};
}

struct AccelerationProfile 
{
    Real acc_time;
    Real dec_time;
    Real still_time;
    Real acceleration;
    Real deceleration;
    Real blend_time = 0.25;

    Real get_acceleration(Real time) const 
    {
        static auto smoothstep = [](Real edge0, Real edge1, Real x) -> Real 
        {
            x = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
            return x * x * (3 - 2 * x);
        };

        static auto blend = [](Real phase_start, Real blend_time, Real time, Real from, Real to) 
        {
            if (time < phase_start + blend_time) 
            {
                Real t = smoothstep(phase_start, phase_start + blend_time, time);
                return from + t * (to - from);
            }
            return to;
        };

        if (time < acc_time) 
            return blend(0.0, blend_time, time, 0.0, acceleration);
        
        if (time < acc_time + dec_time) 
            return blend(acc_time, blend_time, time, acceleration, deceleration);
        
        if (time < acc_time + dec_time + still_time) 
            return blend(acc_time + dec_time, blend_time, time, deceleration, 0.0);
        
        return 0.0;
    }

    bool is_complete(Real time) const 
    {
        return time >= acc_time + dec_time + still_time;
    }
};

AABB getSceneAABB()
{
    AABB aabb = scene.getRigidObject(0).aabb;
    for (Index i=0; i<scene.rigid_objects.size(); i++) 
    {
        RigidBox &box = scene.getRigidObject(i);
        aabb.expand(box.aabb.min);
        aabb.expand(box.aabb.max);
    }

    return aabb;
}

struct PrepareSceneOutput
{
    AABB stack_aabb;
    int last_layer_indexes[2];
};

PrepareSceneOutput prepare_scene()
{
    scene.clear();

    int last_layer_idxs[2];

    Box bpallet = load_rigid_schema("palleting_data\\" + schema_folder, scale_factor, last_layer_idxs);

    AABB stack_aabb = getSceneAABB();
    Real3 center    = (stack_aabb.min + stack_aabb.max) * 0.5;

    Real length_x = stack_aabb.max.x - stack_aabb.min.x;
    Real length_y = stack_aabb.max.y - stack_aabb.min.y;
    Real length_z = stack_aabb.max.z - stack_aabb.min.z;

    Real slitta_height = 0.10 * scale_factor;
    Real pallet_height = 0.13 * scale_factor;

    // PALLET HITBOX
    scene.addRigidObject(
            RigidBox(
                Real3(center.x, -2.0 - bpallet.size.y/2.0 - 0.001, center.z),
                Real3(bpallet.size.x + 1.0, bpallet.size.y, bpallet.size.z + 1.0),
                1.0));
    scene.getRigidObject(scene.rigid_objects.size()-1).make_static();

    // GROUND
    scene.addRigidObject(
            RigidBox(
                Real3(28.5, - 3.0 - pallet_height - slitta_height,  center.z),
                Real3(62.0, 2.0,  bpallet.size.z + 4.0),
                1.0));
    scene.getRigidObject(scene.rigid_objects.size()-1).make_static();

    auto whichBox = [&](Real3 p) -> Index 
    {
        for (Index i=0; i<scene.rigid_objects.size()-1; i++) 
        {
            if (scene.getRigidObject(i).aabb.contains(p, 1e-4)) return i;
        }
        return scene.rigid_objects.size()-1;
    };

    auto validBox = [&](Index i) -> bool 
    {
        return i < scene.rigid_objects.size()-2;
    };

    auto attachPoint = [&](Real3 p, Index i) -> Real3 {
        RigidBox &box = scene.getRigidObject(i);
        return p - box.position;
    };

    auto wrapPlane = [&](Real3 p1, Real3 p2, Real3 step1, Real3 step2, uint8_t steps, bool both_directions = true) 
    {
        int gen_cons = 0;

        for (uint8_t i=0; i<=steps; i++) {
        for (uint8_t j=0; j<=steps; j++) {

            Real step1_size = glm::length(step1);
            Real step2_size = glm::length(step2);
            
            Real3 p  = p1 + step1 * ((Real) i) + step2 * ((Real) j);
            Real3 pr = p  + step1;
            Real3 pd = p  + step2;

            Index bi  = whichBox(p);
            Index bir = whichBox(pr);
            Index bid = whichBox(pd);

            Real3 a  = attachPoint(p,  bi);
            Real3 ar = attachPoint(pr, bir);
            Real3 ad = attachPoint(pd, bid);

            RigidBox *b  = validBox(bi)  ? &scene.getRigidObject(bi)  : nullptr;
            RigidBox *br = validBox(bir) ? &scene.getRigidObject(bir) : nullptr;
            RigidBox *bd = validBox(bid) ? &scene.getRigidObject(bid) : nullptr;

            if (validBox(bi) && validBox(bir)) {
                scene.addRigidConstraint(
                        RigidSpringConstraint(
                            wrap_compliance, 
                            b,  br, 
                            a,  ar, 
                            step1_size));
                if (bi != bir) gen_cons++;
            }

            if (both_directions && validBox(bi) && validBox(bid)) {
                scene.addRigidConstraint(
                        RigidSpringConstraint(
                            wrap_compliance, 
                            b,  bd, 
                            a,  ad, 
                            step2_size));
                if (bi != bid) gen_cons++;
            }
        }}

        // std::cout << "Generated " << gen_cons << " wrap constraints.\n";
    };

    auto wrapPlaneRandom = [&](Real3 p1, Real3 p2, int num_constraints) 
    {
        Real step_x = p2.x - p1.x;
        Real step_y = p2.y - p1.y;
        Real step_z = p2.z - p1.z;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<Real> dis(0.0, 1.0);

        int gen_cons = 0;

        while (gen_cons < num_constraints) 
        {
            Real u1 = dis(gen);
            Real v1 = dis(gen);
            Real w1 = dis(gen);

            Real u2 = dis(gen);
            Real v2 = dis(gen);
            Real w2 = dis(gen);

            Real3 pr1 = p1 + Real3(u1 * step_x, v1 * step_y, w1 * step_z);
            Real3 pr2 = p1 + Real3(u2 * step_x, v2 * step_y, w2 * step_z);

            Index b1 = whichBox(pr1);
            Index b2 = whichBox(pr2);
            
            Real3 a1 = attachPoint(pr1, b1);
            Real3 a2 = attachPoint(pr2, b2);

            RigidBox *bx1 = validBox(b1) ? &scene.getRigidObject(b1) : nullptr;
            RigidBox *bx2 = validBox(b2) ? &scene.getRigidObject(b2) : nullptr;

            if (validBox(b1) && validBox(b2) && (b1 != b2)) 
            {
                Real distance = glm::length(pr1 - pr2);
                scene.addRigidConstraint(
                    RigidSpringConstraint(
                        wrap_compliance, 
                        bx1, bx2, 
                        a1,  a2, 
                        distance));
                gen_cons++;
            }
        }

        // std::cout << "Generated " << gen_cons << " wrap constraints.\n";
    };

    auto wrapPlaneRandomLength = [&](Real3 p1, Real3 p2, Real length, int num_constraints, int plane_axis) {
        
        Real step_x = p2.x - p1.x;
        Real step_y = p2.y - p1.y;
        Real step_z = p2.z - p1.z;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<Real> dis(0.0, 1.0);

        int gen_cons = 0;
        
        for (int k = 0; k < num_constraints; k++) 
        {

            // Real u1 = dis(gen);
            // Real v1 = dis(gen);
            // Real w1 = dis(gen);
            Real3 pr1; // = p1 + Real3(u1 * step_x, v1 * step_y, w1 * step_z);
            Real3 pr2;

            while(true) { 
                Real u1 = dis(gen);
                Real v1 = dis(gen);
                Real w1 = dis(gen);
                pr1     = p1 + Real3(u1 * step_x, v1 * step_y, w1 * step_z);

                Real rand_direction = dis(gen) * 2.0 * glm::pi<Real>();
                Real cos_length = glm::cos(rand_direction) * length;
                Real sin_length = glm::sin(rand_direction) * length;
                switch(plane_axis) {
                    case 0: pr2 = pr1 + Real3(0.0, cos_length, sin_length); break;
                    case 1: pr2 = pr1 + Real3(cos_length, 0.0, sin_length); break;
                    case 2: pr2 = pr1 + Real3(cos_length, sin_length, 0.0); break;
                }
                bool inside = (pr2.x >= p1.x && pr2.x <= p2.x &&
                               pr2.y >= p1.y && pr2.y <= p2.y &&
                               pr2.z >= p1.z && pr2.z <= p2.z);

                if (!inside) continue;

                Index b1 = whichBox(pr1);
                Index b2 = whichBox(pr2);

                if (validBox(b1) && validBox(b2) && (b1 != b2)) break;
            }

            
            Index b1 = whichBox(pr1);
            Index b2 = whichBox(pr2);
            
            Real3 a1 = attachPoint(pr1, b1);
            Real3 a2 = attachPoint(pr2, b2);

            RigidBox *bx1  = validBox(b1)  ? &scene.getRigidObject(b1)  : nullptr;
            RigidBox *bx2  = validBox(b2)  ? &scene.getRigidObject(b2) : nullptr;

            if (validBox(b1) && validBox(b2) && (b1 != b2)) 
            {
                Real distance = glm::length(pr1 - pr2);
                scene.addRigidConstraint(
                    RigidSpringConstraint(
                        wrap_compliance, 
                        bx1, bx2, 
                        a1, a2, 
                        distance));
                gen_cons++;
            }
        }

        // std::cout << "Generated " << gen_cons << " wrap constraints.\n";
    };

    auto transform_axis_aligned_rectangle = [](Real3& p1,  Real3& p2, Real scale, Real rotation_angle, int plane_axis) 
    {    
        Real3 center = (p1 + p2) * 0.5;

        p1 -= center;
        p2 -= center;

        p1 *= scale;
        p2 *= scale;
        
        Real4x4 transform = Real4x4(1.0f);

        Real angle_rad = glm::radians(rotation_angle);
        if      (plane_axis == 0) transform = glm::rotate(transform, angle_rad, Real3(1.0f, 0.0f, 0.0f));
        else if (plane_axis == 1) transform = glm::rotate(transform, angle_rad, Real3(0.0f, 1.0f, 0.0f));
        else                      transform = glm::rotate(transform, angle_rad, Real3(0.0f, 0.0f, 1.0f));

        Real4 p1t = transform * Real4(p1, 1.0f);
        Real4 p2t = transform * Real4(p2, 1.0f);
        
        p1 = Real3(p1t.x, p1t.y, p1t.z);
        p2 = Real3(p2t.x, p2t.y, p2t.z);

        p1 += center;
        p2 += center;
    };

    auto rotate_around_origin = [](Real3& point, Real angle_degrees, int plane_axis) {
        Real angle_rad = glm::radians(angle_degrees);
        Real cos_angle = std::cos(angle_rad);
        Real sin_angle = std::sin(angle_rad);
        
        if (plane_axis == 0) { 
            Real y = point.y * cos_angle - point.z * sin_angle;
            Real z = point.y * sin_angle + point.z * cos_angle;
            point.y = y;
            point.z = z;
        } else if (plane_axis == 1) { 
            Real x = point.x * cos_angle + point.z * sin_angle;
            Real z = -point.x * sin_angle + point.z * cos_angle;
            point.x = x;
            point.z = z;
        } else { 
            Real x = point.x * cos_angle - point.y * sin_angle;
            Real y = point.x * sin_angle + point.y * cos_angle;
            point.x = x;
            point.y = y;
        }
    };

    auto wrapPlaneRotated = [&](Real3 p1, Real3 p2, Real3 step1, Real3 step2, uint8_t steps, Real angle, int plane_axis) 
    {

        Real3 c1p1 = p1;
        Real3 c1p2 = p2;
        Real3 c1step1 = step1;
        Real3 c1step2 = step2;
        transform_axis_aligned_rectangle(c1p1, c1p2, 3.0, angle, plane_axis);
        rotate_around_origin(c1step1, angle, plane_axis);
        rotate_around_origin(c1step2, angle, plane_axis);
        steps += steps*2;

        wrapPlane(c1p1, c1p2, c1step1, c1step2, steps);
    };

    auto wrapPlaneRotatedMirrored = [&](Real3 p1, Real3 p2, Real3 step1, Real3 step2, uint8_t steps, Real angle, int plane_axis) 
    {

        Real3 c1p1 = p1;
        Real3 c1p2 = p2;
        Real3 c1step1 = step1;
        Real3 c1step2 = step2;
        transform_axis_aligned_rectangle(c1p1, c1p2, 3.0, angle, plane_axis);
        rotate_around_origin(c1step1, angle, plane_axis);
        rotate_around_origin(c1step2, angle, plane_axis);
        
        Real3 c2p1 = p1;
        Real3 c2p2 = p2;
        Real3 c2step1 = step1;
        Real3 c2step2 = step2;
        transform_axis_aligned_rectangle(c2p1, c2p2, 3.0, -angle, plane_axis);
        rotate_around_origin(c2step1, -angle, plane_axis);
        rotate_around_origin(c2step2, -angle, plane_axis);

        steps += steps*2;

        wrapPlane(c1p1, c1p2, c1step1, c1step2, steps, false);
        wrapPlane(c2p1, c2p2, c2step1, c2step2, steps, false);
    };

    auto wrapPlaneShifted = [&](Real3 p1, Real3 p2, Real3 step1, Real3 step2, uint8_t steps, Real shift) 
    {

        p1 += step1 * shift + step2 * shift;
        p2 += step1 * shift + step2 * shift;

        wrapPlane(p1, p2, step1, step2, steps);
    };

    auto wrapPlaneEdges = [&](Real3 p1, Real3 p2, Real3 step1, Real3 step2, uint8_t steps, Real shift) 
    {
        // Opposite edges: B1 (p1→p1+step1*N) and B3 (p2→p2-step1*N)
        for (uint8_t i = 0; i <= steps; i++) 
        {

            Real t  = (Real) i / (Real) steps;
            Real to = (Real) (steps - i) / (Real) steps;

            Real3 e1 = p1 + step1 * (t  * (Real)steps);                // Edge 1 point
            Real3 e3 = p2 - step1 * (to * (Real)steps) + step1*shift;  // Opposite edge point

            Index b1 = whichBox(e1);
            Index b3 = whichBox(e3);

            if (validBox(b1) && validBox(b3)) 
            {
                Real3 a1  = attachPoint(e1, b1);
                Real3 a3  = attachPoint(e3, b3);
                Real dist = glm::length(e1 - e3);

                scene.addRigidConstraint(
                        RigidSpringConstraint(
                            wrap_compliance,
                            &scene.getRigidObject(b1),
                            &scene.getRigidObject(b3),
                            a1, a3, dist));
            }
        }
    };

    auto attachBase = [&](Real3 p1, Real3 p2) 
    {
        for (Index bi=0; bi<scene.rigid_objects.size()-1; bi++) 
        {
            RigidBox &box = scene.getRigidObject(bi);
            for (int vi=0; vi<8; vi++) {
                Real3 v = box.world_vertices[vi];

                if ( (v.x >= std::min(p1.x, p2.x) - 1e-4) && (v.x <= std::max(p1.x, p2.x) + 1e-4) &&
                     (v.z >= std::min(p1.z, p2.z) - 1e-4) && (v.z <= std::max(p1.z, p2.z) + 1e-4) &&
                     (std::abs(v.y - p1.y) < 1e-4) ) 
                {
                    
                    scene.addRigidConstraint(
                            FixedRigidSpringConstraint(
                                base_attach_compliance, 
                                &box, 
                                box.body_vertices[vi], 
                                v,
                                0.0));
                }
            }   
        }
    };

    auto applyWrapType = [&](
        const std::string& type, int steps, Real param, 
        const Real3& p0, const Real3& p1, const Real3& p2, const Real3& p3, 
        const Real3& p4, const Real3& p5, const Real3& p6, const Real3& p7,
        const Real3& xstep, const Real3& ystep, const Real3& zstep,
        Real length_x, Real length_y, Real length_z) 
    {
        if (type == NONE) return;

        if (type == ROTATED) 
        {
            wrapPlaneRotated(p1, p6, xstep, ystep, steps, param, 2);
            wrapPlaneRotated(p0, p7, xstep, ystep, steps, param, 2);
            wrapPlaneRotated(p0, p5, zstep, ystep, steps, param, 0);
            wrapPlaneRotated(p3, p6, zstep, ystep, steps, param, 0);
            wrapPlaneRotated(p4, p6, xstep, zstep, steps, param, 1);
        }
        else if (type == ROTATED_MIRRORED) 
        {
            wrapPlaneRotatedMirrored(p1, p6, xstep, ystep, steps, param, 2);
            wrapPlaneRotatedMirrored(p0, p7, xstep, ystep, steps, param, 2);
            wrapPlaneRotatedMirrored(p0, p5, zstep, ystep, steps, param, 0);
            wrapPlaneRotatedMirrored(p3, p6, zstep, ystep, steps, param, 0);
            wrapPlaneRotatedMirrored(p4, p6, xstep, zstep, steps, param, 1);
        }
        else if (type == GRID) 
        {
            wrapPlane(p1, p6, xstep, ystep, steps);
            wrapPlane(p0, p7, xstep, ystep, steps);
            wrapPlane(p0, p5, zstep, ystep, steps);
            wrapPlane(p3, p6, zstep, ystep, steps);
            wrapPlane(p4, p6, xstep, zstep, steps);
        }
        else if (type == SHIFTED) 
        {
            wrapPlaneShifted(p1, p6, xstep, ystep, steps, param);
            wrapPlaneShifted(p0, p7, xstep, ystep, steps, param);
            wrapPlaneShifted(p0, p5, zstep, ystep, steps, param);
            wrapPlaneShifted(p3, p6, zstep, ystep, steps, param);
            wrapPlaneShifted(p4, p6, xstep, zstep, steps, param);
        }
        else if (type == RANDOM) 
        {
            wrapPlaneRandom(p1, p6, steps);
            wrapPlaneRandom(p0, p7, steps);
            wrapPlaneRandom(p0, p5, steps);
            wrapPlaneRandom(p3, p6, steps);
            wrapPlaneRandom(p4, p6, steps);
        }
        else if (type == RANDOM_LENGTH) 
        {
            Real clamp_p = glm::clamp((Real)param, 0.0, 0.9);
            wrapPlaneRandomLength(p1, p6, length_x * clamp_p, steps, 2);
            wrapPlaneRandomLength(p0, p7, length_x * clamp_p, steps, 2);
            wrapPlaneRandomLength(p0, p5, length_z * clamp_p, steps, 0);
            wrapPlaneRandomLength(p3, p6, length_z * clamp_p, steps, 0);
            wrapPlaneRandomLength(p4, p6, length_x * clamp_p, steps, 1);
        }
        else if (type == EDGES) 
        {
            wrapPlaneEdges(p1, p6, ystep, xstep, steps, param);
            wrapPlaneEdges(p0, p7, ystep, xstep, steps, param);
            wrapPlaneEdges(p0, p5, ystep, zstep, steps, param);
            wrapPlaneEdges(p3, p6, ystep, zstep, steps, param);
            wrapPlaneEdges(p4, p6, xstep, zstep, steps, param);
        }
    };

    Real3 p0(stack_aabb.min.x, stack_aabb.min.y, stack_aabb.min.z);
    Real3 p1(stack_aabb.min.x, stack_aabb.min.y, stack_aabb.max.z);
    Real3 p2(stack_aabb.max.x, stack_aabb.min.y, stack_aabb.max.z);
    Real3 p3(stack_aabb.max.x, stack_aabb.min.y, stack_aabb.min.z);

    Real3 p4(stack_aabb.min.x, stack_aabb.max.y, stack_aabb.min.z);
    Real3 p5(stack_aabb.min.x, stack_aabb.max.y, stack_aabb.max.z);
    Real3 p6(stack_aabb.max.x, stack_aabb.max.y, stack_aabb.max.z);
    Real3 p7(stack_aabb.max.x, stack_aabb.max.y, stack_aabb.min.z);

    auto run_wrap = [&](std::string type, int steps, Real param) 
    {
        if (type == NONE) return;

        Real3 xstep(length_x / (Real)steps, 0.0, 0.0);
        Real3 ystep(0.0, length_y / (Real)steps, 0.0);
        Real3 zstep(0.0, 0.0, length_z / (Real)steps);

        applyWrapType(type, steps, param, p0, p1, p2, p3, p4, p5, p6, p7, xstep, ystep, zstep, length_x, length_y, length_z);
    };

    run_wrap(wrap_type,     wrap_steps,     wrap_param);
    run_wrap(wrap_type_sec, wrap_steps_sec, wrap_param_sec);

    attachBase(p0, p2);

    scene.addSceneObject(load_scene_object_from_obj("..\\..\\assets\\slitta.obj", scale_factor));
    scene.addSceneObject(load_scene_object_from_obj("..\\..\\assets\\pallet.obj", scale_factor));

    RigidBox &pallet_hitbox = scene.rigid_objects[scene.rigid_objects.size()-2];
    SceneObject &slitta = scene.scene_objects[0];
    SceneObject &pallet = scene.scene_objects[1];

    slitta.translate(Real3(center.x, -2.0 - pallet_height, center.z));
    pallet.translate(Real3(center.x, -2.0, center.z));

    return {stack_aabb, {last_layer_idxs[0], last_layer_idxs[1]}};
}

void rigid_world_schema() 
{
    std::ofstream fout("..\\..\\animation\\camera_x.txt", std::ios::out | std::ios::trunc);
    if (!fout.is_open()) { std::cerr << "Errore apertura file!" << std::endl; return; }

    uint64_t step;
    Real     time;

    Real total_physics_time;
    AccelerationProfile profile;
    Real3 vel_vector;

    AABB  stack_aabb;
    Real3 center;

    RigidBox *pallet_hitbox;

    
    Real base_x, base_y;

    int SLOWING_FACTOR;

    auto exportFrameToObj = [&](uint64_t step, Real3 center)
    {
        export_scene_to_obj(scene, (step / (frequency/SLOWING_FACTOR)), scale_factor, -center, prefix);
        if (export_stretch_perc)
            export_wrap_displacement_to_obj(scene,  (step / (frequency/SLOWING_FACTOR)), tearing_stretch_percentage, scale_factor, -center, prefix);
        else
            export_wrap_to_obj(scene,  (step / (frequency/SLOWING_FACTOR)), scale_factor, -center, prefix);
        fout << center.x / scale_factor << "\n";
    };

    auto reset_state = [&]()
    {
        total_physics_time = 0.0;
        vel_vector         = Real3(0.0);
        profile            = {acc_time, dec_time, still_time, acceleration, deceleration};
        step               = 0;
        time               = 0.0;

        auto [stack_aabb, last_layer_idxs] = prepare_scene();
        center     = (stack_aabb.min + stack_aabb.max) * 0.5;
        center.x   = 0.0;

        size_t num_data_points = (acc_time + dec_time + still_time) * DataCollection::DataPointsPerSecond; 

        data.init(scene, stack_aabb, last_layer_idxs, num_data_points);
        base_x = stack_aabb.min.x;
        base_y = stack_aabb.min.y;

        XPBD_init(xpbd_steps_x_second, xpbd_iters_x_step);

        rigid_spring_renderer.init(scene);
        fixed_rigid_spring_renderer.init(scene);

        pallet_hitbox = &scene.rigid_objects[scene.rigid_objects.size()-2];

        SLOWING_FACTOR = video_fps;
    };

    reset_state();

    while (!glfwWindowShouldClose(window)) 
    {
        if (start_simulation) { start_simulation = false; }

        if (app_state == AppState::RUNNING)
        {
            if (export_obj && (step % (frequency/SLOWING_FACTOR) == 0)) exportFrameToObj(step, center);

            time = step * delta_t;

            if (profile.is_complete(time)) 
            {
                end_simulation = true;
                app_state      = AppState::FINISHED;
            }

            Real3 acc_vector = Real3(profile.get_acceleration(time), 0.0, 0.0);

            vel_vector  += acc_vector * delta_t;
            Real3 offset = vel_vector * delta_t;

            for (FixedRigidSpringConstraint &c : scene.fixed_rigid_constraints) c.world_attach += offset;
            center += offset;
            base_x += offset.x;
            pallet_hitbox->translate(offset);

            MEASURE_TIME(XPBD_step(scene), total_physics_time);

            if (apply_tearing && (step % (frequency / 60) == 0))
            {
                for (auto& constraint : scene.rigid_constraints) 
                {   
                    Real curr_length    = getLength(constraint);
                    Real stretch        = curr_length - constraint.rest_length;
                    Real tear_threshold = constraint.rest_length * tearing_stretch_percentage;

                    if (stretch > tear_threshold) constraint.active = false;

                    // Real force_magnitude = constraint.lambda / (delta_t*delta_t);
                    // if (std::abs(force_magnitude) > force_tearing_threshold) constraint.active = false;
                }
            }

            if (collect_data && step % (frequency / DataCollection::DataPointsPerSecond) == 0)
                data.update(scene, time, center, base_x, base_y, acc_vector);

            step++;
        }

        if (app_state != AppState::RUNNING || step % (frequency/60) == 0) 
        {
            loop_init();
            render_ui(step, time, total_physics_time); 
            rendering(Real3(center.x, center.y, center.z)); 
            loop_terminate();
        }

        if (end_simulation)
        {
            end_simulation = false;
            if (collect_data) data.print();
        }

        if (reset_simulation) 
        {
            reset_simulation = false;
            reset_state();
        }
    }

    fout.close();
}

int main(int argc, char* argv[]) 
{
    parseArgument(argc, argv);

    if (!graphics_init()) return -1; 

    /*
    constexpr int STEPS = 4;
    constexpr int COMPL = 1;
    constexpr int PARAM = 4;

    uint8_t steps[STEPS]         = {20, 30, 50, 100};
    Real coll_compliances[COMPL] = {0.00005};
    Real wrap_params[PARAM]      = {0.1, 0.3, 0.5, 0.7};

    for (int i=0; i<STEPS; i++) {
        for (int j=0; j<COMPL; j++) {
            for (int k=0; k<PARAM; k++) {
                wrap_steps      = steps[i];
                wrap_compliance = coll_compliances[j];
                wrap_param      = wrap_params[k];
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << wrap_param;
                prefix          = schema_folder + "_" + wrap_type + "_" + std::to_string(wrap_steps) + "_" + oss.str();
                std::cout << prefix << std::endl;
                rigid_world_schema();
            }
        }
    }

    rigid_world_schema_stretch_stats(50);
    */

    load_configuration_file("..\\..\\configurations\\c1.conf");

    rigid_world_schema();

    graphics_close();

    return 0;
}
