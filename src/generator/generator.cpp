#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
#include <limits>
#include <sstream>

#include "spectral.h"
#include "convert_obj.h"

#include "runtime/common.h"
#include "runtime/file_path.h"

inline Target cpuid()
{
    std::ifstream info(CPUINFO_PATH);
    if (!info)
        return Target(0);

    std::string line;
    std::vector<std::string> isa_list{
        "asimd", "sse4_2", "avx", "avx2"};
    std::unordered_set<std::string> detected;
    while (std::getline(info, line))
    {
        for (auto isa : isa_list)
        {
            if (line.find(isa) != std::string::npos)
                detected.insert(isa);
        }
    }
    if (detected.count("avx2"))
        return Target::AVX2;
    if (detected.count("avx"))
        return Target::AVX;
    if (detected.count("sse4_2"))
        return Target::SSE42;
    if (detected.count("asimd"))
        return Target::ASIMD;
    return Target::GENERIC;
}

static void usage()
{
    std::cout << "converter [options] file\n"
              << "Available options:\n"
              << "    -h     --help                Shows this message\n"
              << "    -t     --target              Sets the target platform (default: autodetect CPU)\n"
              << "    -d     --device              Sets the device to use on the selected platform (default: 0)\n"
              << "           --max-path-len        Sets the maximum path length (default: 64)\n"
              << "    -spp   --samples-per-pixel   Sets the number of samples per pixel (default: 4)\n"
              << "           --fusion              Enables megakernel shader fusion (default: disabled)\n"
#ifdef ENABLE_EMBREE_BVH
              << "           --embree-bvh          Use Embree to build the BVH (default: disabled)\n"
#endif
              << "Available targets:\n"
              << "    generic, sse42, avx, avx2, avx2-embree, asimd,\n"
              << "    nvvm = nvvm-streaming, nvvm-megakernel,\n"
              << "    amdgpu = amdgpu-streaming, amdgpu-megakernel\n"
              << std::flush;
}

static bool check_option(int i, int argc, char **argv)
{
    if (i + 1 >= argc)
    {
        std::cerr << "Missing argument for '" << argv[i] << "'. Aborting." << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cerr << "Not enough arguments. Run with --help to get a list of options." << std::endl;
        return 1;
    }

    std::string input_file;
    size_t dev = 0;
    size_t spp = 4;
    size_t max_path_len = 64;
    auto target = Target::INVALID;
    bool embree_bvh = false;
    bool fusion = false;
    for (int i = 1; i < argc; ++i)
    {
        if (argv[i][0] == '-')
        {
            if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
            {
                usage();
                return 0;
            }
            else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--target"))
            {
                if (!check_option(i++, argc, argv))
                    return 1;
                if (!strcmp(argv[i], "sse42"))
                    target = Target::SSE42;
                else if (!strcmp(argv[i], "avx"))
                    target = Target::AVX;
                else if (!strcmp(argv[i], "avx2"))
                    target = Target::AVX2;
                else if (!strcmp(argv[i], "avx2-embree"))
                    target = Target::AVX2_EMBREE;
                else if (!strcmp(argv[i], "asimd"))
                    target = Target::ASIMD;
                else if (!strcmp(argv[i], "nvvm") || !strcmp(argv[i], "nvvm-streaming"))
                    target = Target::NVVM_STREAMING;
                else if (!strcmp(argv[i], "nvvm-megakernel"))
                    target = Target::NVVM_MEGAKERNEL;
                else if (!strcmp(argv[i], "amdgpu") || !strcmp(argv[i], "amdgpu-streaming"))
                    target = Target::AMDGPU_STREAMING;
                else if (!strcmp(argv[i], "amdgpu-megakernel"))
                    target = Target::AMDGPU_MEGAKERNEL;
                else if (!strcmp(argv[i], "generic"))
                    target = Target::GENERIC;
                else
                {
                    std::cerr << "Unknown target '" << argv[i] << "'. Aborting." << std::endl;
                    return 1;
                }
            }
            else if (!strcmp(argv[i], "-d") || !strcmp(argv[i], "--device"))
            {
                if (!check_option(i++, argc, argv))
                    return 1;
                dev = strtoul(argv[i], NULL, 10);
            }
            else if (!strcmp(argv[i], "-spp") || !strcmp(argv[i], "--samples-per-pixel"))
            {
                if (!check_option(i++, argc, argv))
                    return 1;
                spp = strtol(argv[i], NULL, 10);
            }
            else if (!strcmp(argv[i], "--max-path-len"))
            {
                if (!check_option(i++, argc, argv))
                    return 1;
                max_path_len = strtol(argv[i], NULL, 10);
            }
            else if (!strcmp(argv[i], "--fusion"))
            {
                fusion = true;
#ifdef ENABLE_EMBREE_BVH
            }
            else if (!strcmp(argv[i], "--embree-bvh"))
            {
                embree_bvh = true;
#endif
            }
            else
            {
                std::cerr << "Unknown option '" << argv[i] << "'. Aborting." << std::endl;
                return 1;
            }
        }
        else
        {
            if (input_file != "")
            {
                std::cerr << "Only one file can be converted. Aborting." << std::endl;
                return 1;
            }
            input_file = argv[i];
        }
    }

    if (fusion && target != Target::NVVM_MEGAKERNEL && target != Target::AMDGPU_MEGAKERNEL) {
        std::cerr << "Fusion is only available for megakernel targets. Aborting." << std::endl;
        return 1;
    }

    if (input_file == "") {
        std::cerr << "Please specify an input file to convert. Aborting." << std::endl;
        return 1;
    }

    if (target == Target::INVALID) {
        target = cpuid();
        if (target == Target::GENERIC)
            warn("No vector instruction set detected. Select the target platform manually to improve performance.");
    }

    std::unique_ptr<SpectralUpsampler> upsampler;
    try {
        upsampler = std::make_unique<SpectralUpsampler>("srgb.coeff");
    }
    catch (const std::exception &e) {
        error("Spectral Upsampler: ", e.what());
        return 1;
    }

    std::ofstream of("main.impala");
    FilePath input_path(input_file);
    if(input_path.extension() == "obj") {
        if (!convert_obj(input_file, target, dev, max_path_len, spp, embree_bvh, fusion, upsampler.get(), of))
            return 1;
    } else {
        error("Unknown input file");
        return 1;
    }
    return 0;
}
