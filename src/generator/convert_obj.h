#pragma once

#include "target.h"
#include <string>

class SpectralUpsampler;
bool convert_obj(const std::string &file_name, Target target,
                size_t dev, size_t max_path_len, size_t spp, bool embree_bvh, bool fusion, 
                SpectralUpsampler* upsampler, std::ostream &os);