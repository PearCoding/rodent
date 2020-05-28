#include <cmath>

#include "image.h"

void gamma_correct(ImageRgba32& img) {
    for (size_t y = 0; y < img.height; ++y) {
        for (size_t x = 0; x < img.width; ++x) {
            auto* pix = &img.pixels[4 * (y * img.width + x)];
            for (int i = 0; i < 3; ++i)
                pix[i] = std::pow(pix[i], 2.2f);
        }
    }
}