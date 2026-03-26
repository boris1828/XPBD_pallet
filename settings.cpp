#pragma once

#include "types.h"
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

using string = std::string;

unsigned int WIDTH  = 1600;
unsigned int HEIGHT = 800;
bool DO_VIDEO       = false;
double targetFPS    = 25*4*60.0; // 4*60.0;
bool BOX_DRAW_ONLY_EXTERNAL_EDGES = true;

// string GRID             = "grid";
// string NONE             = "none";
// string ROTATED          = "rotated";
// string ROTATED_MIRRORED = "rotated_mirrored";
// string SHIFTED          = "shifted";
// string RANDOM           = "random";
// string RANDOM_LENGTH    = "random_length";
// string EDGES            = "edges";

#define WRAP_MODES \
    X(NONE,             none)             \
    X(GRID,             grid)             \
    X(SHIFTED,          shifted)          \
    X(ROTATED,          rotated)          \
    X(ROTATED_MIRRORED, rotated_mirrored) \
    X(RANDOM_LENGTH,    random_length)    \
    X(RANDOM,           random)           \
    X(EDGES,            edges)            \

#define X(name, string_val) string name = #string_val;
WRAP_MODES
#undef X

#define CONFIG_PARAMS                            \
    X(Real,   coll_compliance,            0.0001)    \
    X(Real,   box_edge_compliance,        0.00005)   \
    X(Real,   volume_compliance,          0.0005)    \
    X(Real,   wrap_compliance,            0.00002)   \
    X(Real,   base_attach_compliance,     0.000005)  \
    X(int,    wrap_steps,                 10)        \
    X(int,    wrap_steps_sec,             10)        \
    X(Real,   wrap_param,                 0.0)       \
    X(Real,   wrap_param_sec,             0.0)       \
    X(string, wrap_type,                  GRID)      \
    X(string, wrap_type_sec,              NONE)      \
    X(string, schema_folder,              "schema1") \
    X(bool,   look_at_stack_center,       false)     \
    X(Real,   camera_xoff,                -0.3)      \
    X(Real,   camera_yoff,                0.0)       \
    X(Real,   camera_zoff,                8.0)       \
    X(Real,   mu_dynamic,                 0.5)       \
    X(Real,   acceleration,               10.0)      \
    X(Real,   deceleration,               -10.0)     \
    X(Real,   acc_time,                   2.0)       \
    X(Real,   dec_time,                   2.0)       \
    X(Real,   still_time,                 1.0)       \
    X(string, prefix,                     "")        \
    X(bool,   export_obj,                 false)     \
    X(bool,   collect_data,               false)     \
    X(bool,   apply_tearing,              false)     \
    X(bool,   render_tearing,             false)     \
    X(Real,   tearing_stretch_percentage, 0.01)      \
    X(Real,   scale_factor,               2.0)       \
    X(Real,   force_tearing_threshold,    1.0)       \
    X(bool,   export_stretch_perc,        false)     \
    X(int,    video_fps,                  24*3)      \
    X(int,    xpbd_steps_x_second,        1000)      \
    X(int,    xpbd_iters_x_step,             1)      \

#define X(type, name, def_value) type name = def_value;
CONFIG_PARAMS
#undef X

void load_configuration_file(const std::string& filename) 
{
    std::ifstream file(filename);
    if (!file.is_open()) return;
    
    std::unordered_map<std::string, std::string> env_vars;
    std::string line;

    auto trim = [](std::string& str) {
        str.erase(0, str.find_first_not_of(" \t"));
        str.erase(str.find_last_not_of(" \t") + 1);
    };
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        size_t equal_pos = line.find('=');
        if (equal_pos == std::string::npos) continue;
        
        std::string key   = line.substr(0, equal_pos);
        std::string value = line.substr(equal_pos + 1);
        
        trim(key);
        trim(value);
        
        if (!key.empty()) env_vars[key] = value;
    }
    
    auto update_Real = [&](const std::string& key, Real& var) {
        if (env_vars.count(key)) var = std::stod(env_vars[key]);
    };
    
    auto update_uint8 = [&](const std::string& key, uint8_t& var) {
        if (env_vars.count(key)) var = static_cast<uint8_t>(std::stoi(env_vars[key]));
    };

    auto update_int = [&](const std::string& key, int& var) {
        if (env_vars.count(key)) var = static_cast<int>(std::stoi(env_vars[key]));
    };
    
    auto update_string = [&](const std::string& key, std::string& var) {
        if (env_vars.count(key)) var = env_vars[key];
    };

    auto update_bool = [&](const std::string& key, bool& var) {
        if (env_vars.count(key)) {
            std::string value = env_vars[key];
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            trim(value);
            
            if (value == "true" || value == "1" || value == "yes" || value == "on") 
                var = true;
            else if (value == "false" || value == "0" || value == "no" || value == "off") 
                var = false;
        }
    };
    
    #define X(type, name, defualt) update_##type (#name, name);
    CONFIG_PARAMS
    #undef X
}