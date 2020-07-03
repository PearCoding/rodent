#pragma once

#include <cstdint>

enum class Target : uint32_t
{
    GENERIC = 0,
    AVX2,
    AVX2_EMBREE,
    AVX,
    SSE42,
    ASIMD,
    NVVM_STREAMING,
    NVVM_MEGAKERNEL,
    AMDGPU_STREAMING,
    AMDGPU_MEGAKERNEL,
    INVALID
};
