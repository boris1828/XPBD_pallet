cmake_minimum_required(VERSION 3.15)
project(XPBDPallet)

set(CMAKE_CXX_STANDARD 17)

# set(glfw3_DIR "C:/Users/Workstation/vcpkg/installed/x64-windows/share/glfw3")
# set(glad_DIR  "C:/Users/Workstation/vcpkg/installed/x64-windows/share/glad")
# set(glm_DIR   "C:/Users/Workstation/vcpkg/installed/x64-windows/share/glm")

if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "")
endif()

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_executable(XPBDPallet main.cpp)

target_link_libraries(XPBDPallet
    PRIVATE
    glfw
    glad::glad
    glm::glm
)

# ------------------ Ottimizzazioni in Release ------------------
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        message(STATUS "Applying MSVC Release optimizations: /O2 /GL /LTCG")
        add_compile_options(/GL)
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
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