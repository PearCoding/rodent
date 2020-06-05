#pragma once

#include "runtime/file_path.h"

class SpectralUpsampler;

/// Exports image while upsampling rgb data and returns path to the new generated file
FilePath export_image(SpectralUpsampler* upsampler, const FilePath& path);