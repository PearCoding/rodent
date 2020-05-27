#include "spectral.h"
#include <fstream>
#include <cstring>

// Base on Wenzel Jakob and Johannes Hanika. 2019. A Low-Dimensional Function Space for Efficient Spectral Upsampling.
// In Computer Graphics Forum (Proceedings of Eurographics) 38(2).

constexpr int COEFFS_N = 3; // Has to be 3 all the time!
struct _SpectralUpsamplerInternal
{
    uint32_t Resolution;
    float *Scale;
    float *Data;
};

SpectralUpsampler::SpectralUpsampler(const char *filename)
    : mInternal(new _SpectralUpsamplerInternal())
{
    mInternal->Resolution = 0;
    mInternal->Scale = nullptr;
    mInternal->Data = nullptr;

    std::ifstream stream;
    stream.open(filename, std::ios::in | std::ios::binary);
    if (!stream)
        throw std::runtime_error("Given spectral coefficients file was not found");

    char header[4];
    stream.read(reinterpret_cast<char *>(header), 4 * sizeof(char));

    if (memcmp(header, "SPEC", 4) != 0)
        throw std::runtime_error("Given spectral coefficients file is invalid");

    stream.read(reinterpret_cast<char *>(&mInternal->Resolution), sizeof(mInternal->Resolution));

    const size_t size_scale = mInternal->Resolution;
    const size_t size_data = mInternal->Resolution * mInternal->Resolution * mInternal->Resolution * 3 * COEFFS_N;

    mInternal->Scale = new float[size_scale];
    stream.read(reinterpret_cast<char *>(mInternal->Scale), size_scale * sizeof(float));

    mInternal->Data = new float[size_data];
    stream.read(reinterpret_cast<char *>(mInternal->Data), size_data * sizeof(float));
}

SpectralUpsampler::~SpectralUpsampler()
{
    if (mInternal->Scale)
        delete[] mInternal->Scale;
    if (mInternal->Data)
        delete[] mInternal->Data;
}

static int find_interval(const float *values, int size_, float x)
{
    int left = 0;
    int last_interval = size_ - 2;
    int size = last_interval;

    while (size > 0)
    {
        int half = size >> 1;
        int middle = left + half + 1;

        if (values[middle] < x)
        {
            left = middle;
            size -= half + 1;
        }
        else
        {
            size = half;
        }
    }

    return std::min(left, last_interval);
}

void SpectralUpsampler::prepare(const float *r, const float *g, const float *b, float *out_a, float *out_b, float *out_c, size_t elems)
{
    constexpr float EPS = 0.0001f;
    constexpr float ZERO_A = 0;
    constexpr float ZERO_B = 0;
    constexpr float ZERO_C = -50.0f; // -500 is also possible and is closer to zero, but -50 is sufficient for floats

    const auto res = mInternal->Resolution;
    const auto dx = COEFFS_N;
    const auto dy = COEFFS_N * res;
    const auto dz = COEFFS_N * res * res;

    const float *a_scale = mInternal->Scale;
    const float *a_data = mInternal->Data;

    float coeffs[COEFFS_N];
    for (size_t i = 0; i < elems; ++i)
    {
        const float rgb[3] = {r[i], g[i], b[i]};

        // Handle special case when rgb is zero
        if (rgb[0] <= EPS && rgb[1] <= EPS && rgb[2] <= EPS)
        {
            out_a[i] = ZERO_A;
            out_b[i] = ZERO_B;
            out_c[i] = ZERO_C;
            continue;
        }

        // Determine largest entry
        int largest_entry = 0;
        if (rgb[largest_entry] <= rgb[1])
            largest_entry = 1;
        if (rgb[largest_entry] <= rgb[2])
            largest_entry = 2;

        // Rescale
        float z = rgb[largest_entry];
        float scale = (res - 1) / z;
        float x = rgb[(largest_entry + 1) % 3] * scale;
        float y = rgb[(largest_entry + 2) % 3] * scale;

        // Bilinearly interpolate
        uint32_t xi = std::min((uint32_t)x, res - 2);
        uint32_t yi = std::min((uint32_t)y, res - 2);
        uint32_t zi = find_interval(a_scale, res, z);
        uint32_t off = (((largest_entry * res + zi) * res + yi) * res + xi) * COEFFS_N;

        float x1 = x - xi;
        float x0 = 1.0f - x1;
        float y1 = y - yi;
        float y0 = 1.0f - y1;
        float z1 = (z - a_scale[zi]) / (a_scale[zi + 1] - a_scale[zi]);
        float z0 = 1.0f - z1;

        // Lookup
        for (int j = 0; j < COEFFS_N; ++j)
        {
            coeffs[j] = ((a_data[off] * x0 + a_data[off + dx] * x1) * y0 + (a_data[off + dy] * x0 + a_data[off + dy + dx] * x1) * y1) * z0 + ((a_data[off + dz] * x0 + a_data[off + dz + dx] * x1) * y0 + (a_data[off + dz + dy] * x0 + a_data[off + dz + dy + dx] * x1) * y1) * z1;
            ++off;
        }

        out_a[i] = coeffs[0];
        out_b[i] = coeffs[1];
        out_c[i] = coeffs[2];
    }
}