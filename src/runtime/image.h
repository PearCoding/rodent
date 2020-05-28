#ifndef IMAGE_H
#define IMAGE_H

#include <memory>

#include "file_path.h"

struct ImageRgba32 {
    std::unique_ptr<float[]> pixels;
    size_t width, height;
};

void gamma_correct(ImageRgba32&);

bool load_png(const FilePath&, ImageRgba32&);
bool load_jpg(const FilePath&, ImageRgba32&);
bool load_exr(const FilePath&, ImageRgba32&);

bool save_png(const FilePath&, const ImageRgba32&);
bool save_exr(const FilePath&, const ImageRgba32&, bool specialColorSpace = false);

#endif // IMAGE_H
