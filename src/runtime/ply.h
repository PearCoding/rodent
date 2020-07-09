#pragma once

#include "mesh.h"

namespace ply {
    mesh::TriMesh load_mesh(const std::string& file);
}