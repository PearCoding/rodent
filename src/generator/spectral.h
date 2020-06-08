#pragma once

#include <cmath>
#include <memory>

class SpectralUpsampler {
public:
	explicit SpectralUpsampler(const char* filename);
	~SpectralUpsampler();

	void prepare(const float* r, const float* g, const float* b, float* out_a, float* out_b, float* out_c, size_t elems);

	// slice is the next amount of floats to next element of the channel
	void prepare(const float* r, size_t r_slice, const float* g, size_t g_slice, const float* b, size_t b_slice,
				float* out_a, size_t oa_slice, float* out_b, size_t ob_slice, float* out_c, size_t oc_slice, 
				size_t elems);

	inline static void compute(const float* a, const float* b, const float* c, const float* wavelengths, float* out_weights, size_t elems)
	{
		for (size_t i = 0; i < elems; ++i) {
			const float x  = std::fma(std::fma(a[i], wavelengths[i], b[i]), wavelengths[i], c[i]);
			const float y  = 1.0f / std::sqrt(std::fma(x, x, 1.0f));
			out_weights[i] = std::fma(0.5f * x, y, 0.5f);
		}
	}

	inline static void computeSingle(float a, float b, float c, const float* wavelengths, float* out_weights, size_t elems)
	{	for (size_t i = 0; i < elems; ++i) {
			const float x  = std::fma(std::fma(a, wavelengths[i], b), wavelengths[i], c);
			const float y  = 1.0f / std::sqrt(std::fma(x, x, 1.0f));
			out_weights[i] = std::fma(0.5f * x, y, 0.5f);
		}
	}

private:
	std::unique_ptr<struct _SpectralUpsamplerInternal> mInternal;
};