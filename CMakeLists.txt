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

if(WIN32 AND TARGET glfw)
    add_custom_command(TARGET XPBDPallet POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE_DIR:glfw>/glfw3.dll
        $<TARGET_FILE_DIR:XPBDPallet>
    )
endif()