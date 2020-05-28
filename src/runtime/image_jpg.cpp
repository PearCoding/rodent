#include <fstream>
#include <cmath>
#include <csetjmp>

#include <jpeglib.h>

#include "image.h"

struct enhanced_jpeg_decompress_struct : jpeg_decompress_struct {
    jmp_buf jmp;
    std::istream* is;
    JOCTET src_buf[1024];
};

static void jpeg_error_exit(j_common_ptr cinfo) {
    cinfo->err->output_message(cinfo);
    longjmp(reinterpret_cast<enhanced_jpeg_decompress_struct*>(cinfo)->jmp, 1);
}

static void jpeg_output_message(j_common_ptr) {}

static void jpeg_no_op(j_decompress_ptr) {}

static boolean jpeg_fill_input_buffer(j_decompress_ptr cinfo) {
    auto enhanced = static_cast<enhanced_jpeg_decompress_struct*>(cinfo);
    enhanced->is->read((char*)enhanced->src_buf, 1024);
    cinfo->src->bytes_in_buffer = enhanced->is->gcount();
    cinfo->src->next_input_byte = enhanced->src_buf;
    return TRUE;
}

static void jpeg_skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
    auto enhanced = static_cast<enhanced_jpeg_decompress_struct*>(cinfo);
    if (num_bytes != 0) {
        if (num_bytes < long(cinfo->src->bytes_in_buffer)) {
            cinfo->src->next_input_byte += num_bytes;
            cinfo->src->bytes_in_buffer -= num_bytes;
        } else {
            enhanced->is->seekg(num_bytes - cinfo->src->bytes_in_buffer, std::ios_base::cur);
            cinfo->src->bytes_in_buffer = 0;
        }
    }
}

bool load_jpg(const FilePath& path, ImageRgba32& image) {
    std::ifstream file(path, std::ifstream::binary);
    if (!file)
        return false;

    enhanced_jpeg_decompress_struct cinfo;
    cinfo.is = &file;
    jpeg_error_mgr jerr;

    cinfo.err           = jpeg_std_error(&jerr);
    jerr.error_exit     = jpeg_error_exit;
    jerr.output_message = jpeg_output_message;
    jpeg_create_decompress(&cinfo);

    if (setjmp(cinfo.jmp)) {
        jpeg_abort_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    jpeg_source_mgr src;
    src.init_source       = jpeg_no_op;
    src.fill_input_buffer = jpeg_fill_input_buffer;
    src.skip_input_data   = jpeg_skip_input_data;
    src.resync_to_restart = jpeg_resync_to_restart;
    src.term_source       = jpeg_no_op;
    src.bytes_in_buffer   = 0;
    cinfo.src = &src;

    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);
    image.width  = cinfo.output_width;
    image.height = cinfo.output_height;
    auto image_size = image.width * image.height * 4;
    image.pixels.reset(new float[image_size]);
    std::fill(image.pixels.get(), image.pixels.get() + image_size, 0);
    auto channels = cinfo.output_components;

    std::unique_ptr<JSAMPLE[]> row(new JSAMPLE[image.width * channels]);
    for (size_t y = 0; y < image.height; y++) {
        auto src_ptr = row.get();
        auto dst_ptr = &image.pixels[(image.height - 1 - y) * image.width * 4];
        jpeg_read_scanlines(&cinfo, &src_ptr, 1);
        for (size_t x = 0; x < image.width; ++x, src_ptr += channels, dst_ptr += 4) {
            for (size_t c = 0; c < channels; c++)
                dst_ptr[c] = src_ptr[c] / 255.0f;
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    gamma_correct(image);
    return true;
}
