#include <fstream>
#include <cmath>

#include "image.h"
#include "common.h"

#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

bool load_exr(const FilePath& path, ImageRgba32& img) {
    float* out;
    int width;
    int height;
    const char* err = nullptr;
    
    int ret = LoadEXR(&out, &width, &height, path.path().c_str(), &err);

    if (ret != TINYEXR_SUCCESS) {
        if (err) {
            error("EXR Loading: ", err);
            FreeEXRErrorMessage(err); // release memory of error message.
        }
        return false;
    } else {
        img.width = width;
        img.height = height;
        img.pixels.reset(out);
        return true;
    }
}

bool save_exr(const FilePath& path, const ImageRgba32& img, bool specialColorSpace) {
    EXRHeader header;
    InitEXRHeader(&header);

    EXRImage image;
    InitEXRImage(&image);

    image.num_channels = 4;

    std::vector<float> images[4];
    images[0].resize(img.width * img.height);
    images[1].resize(img.width * img.height);
    images[2].resize(img.width * img.height);
    images[3].resize(img.width * img.height);

    for (int i = 0; i < img.width * img.height; i++) {
        images[0][i] = img.pixels[4*i+0];
        images[1][i] = img.pixels[4*i+1];
        images[2][i] = img.pixels[4*i+2];
        images[3][i] = img.pixels[4*i+3];
    }

    float* image_ptr[3];
    image_ptr[0] = &(images[3].at(0)); // A
    image_ptr[1] = &(images[2].at(0)); // B
    image_ptr[2] = &(images[1].at(0)); // G
    image_ptr[3] = &(images[0].at(0)); // R

    image.images = (unsigned char**)image_ptr;
    image.width = img.width;
    image.height = img.height;

    header.num_channels = image.num_channels;
    header.channels = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
    // Must be (A)BGR order, since most of EXR viewers expect this channel order.
    strncpy(header.channels[0].name, "A", 255); header.channels[0].name[strlen("A")] = '\0';
    strncpy(header.channels[1].name, "B", 255); header.channels[1].name[strlen("B")] = '\0';
    strncpy(header.channels[2].name, "G", 255); header.channels[2].name[strlen("G")] = '\0';
    strncpy(header.channels[3].name, "R", 255); header.channels[3].name[strlen("R")] = '\0';

    if(specialColorSpace) {
        // TODO
    }

    header.pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
    header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
    for (int i = 0; i < header.num_channels; i++) {
      header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of input image
      header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of output image to be stored in .EXR
    }

    const char* err = nullptr;
    int ret = SaveEXRImageToFile(&image, &header, path.path().c_str(), &err);
    if (ret != TINYEXR_SUCCESS) {
        error("EXR Saving: ", err);
        FreeEXRErrorMessage(err); // free's buffer for an error message
        free(header.channels);
        free(header.pixel_types);
        free(header.requested_pixel_types);
        return ret;
    }

    free(header.channels);
    free(header.pixel_types);
    free(header.requested_pixel_types);

    return true;
}
