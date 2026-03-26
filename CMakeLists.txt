cmake_minimum_required(VERSION 3.15)
project(XPBDPallet)

set(CMAKE_CXX_STANDARD 17)

if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "")
endif()

find_package(glfw3 CONFIG REQUIRED)
find_package(glad  CONFIG REQUIRED)
find_package(glm   CONFIG REQUIRED)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_executable(XPBDPallet 
    main.cpp
    ${IMGUI_SOURCES}
)

target_include_directories(XPBDPallet
    PRIVATE
    ${IMGUI_DIR}           
    ${IMGUI_DIR}/backends  
)

target_link_libraries(XPBDPallet
    PRIVATE
    glfw
    glad::glad
    glm::glm
)

# ------------------ Optimization in Release ------------------
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        message(STATUS "Applying MSVC Release optimizations: /O2 /GL ")
        add_compile_options(/O2 /GL /fp:fast)
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(STATUS "Applying GCC/Clang Release optimizations: -O3 -march=native -flto -DNDEBUG")
        add_compile_options(-O3 -march=native -flto -DNDEBUG)
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -flto")
    endif()
endif()
# ---------------------------------------------------------------

if(WIN32 AND TARGET glfw)
    add_custom_command(TARGET XPBDPallet POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE_DIR:glfw>/glfw3.dll
        $<TARGET_FILE_DIR:XPBDPallet>
    )
endif()