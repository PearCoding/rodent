#pragma once

#include "mesh.h"

namespace mts {
    // Load mesh from Mitsuba serialized format
    mesh::TriMesh load_mesh(const std::string& file, size_t shapeIndex = 0);
}