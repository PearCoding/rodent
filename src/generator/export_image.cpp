#include "export_image.h"
#include "runtime/image.h"
#include "runtime/common.h"
#include "spectral.h"

FilePath export_image(SpectralUpsampler *upsampler, const FilePath &path)
{
    ImageRgba32 data;
    const auto ext = path.extension();

    if (ext == "png")
        load_png(path, data);
    else if (ext == "jpg" || ext == "jpeg")
        load_jpg(path, data);
    else if (ext == "exr")
        load_exr(path, data);
    else
    {
        error("Unknown file type '", path.path(), "'");
        return FilePath("");
    }

    upsampler->prepare(&data.pixels[0], 4, &data.pixels[1], 4, &data.pixels[2], 4,
                       &data.pixels[0], 4, &data.pixels[1], 4, &data.pixels[2], 4,
                       data.width * data.height);

    std::string new_path = "data/textures/" + path.remove_extension() + ".exr";
    save_exr(FilePath(new_path), data);
    return new_path;
}