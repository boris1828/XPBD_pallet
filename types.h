#pragma once

#include <glm/glm.hpp>

using float64_t = double;

using vec3 = glm::dvec3;
using mat3 = glm::dmat3;

using Real        = float64_t;
using Real3       = glm::dvec3;
using Real4       = glm::dvec4;
using Quat        = glm::dvec4;
using Real3x3     = glm::dmat3;
using Real4x4     = glm::dmat4;
using TetraIndex  = uint32_t;
using VertexIndex = uint32_t;
using EdgeIndex   = uint32_t;
using Index       = uint32_t;

struct Real3_Color {
    Real x, y, z, r, g, b, a;
};
