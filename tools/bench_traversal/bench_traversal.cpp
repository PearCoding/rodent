#include <iostream>
#include <cstring>
#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>

#include "traversal.h"
#include "load_bvh.h"
#include "load_rays.h"

inline void check_argument(int i, int argc, char** argv) {
    if (i + 1 >= argc) {
        std::cerr << "Missing argument for " << argv[i] << std::endl;
        exit(1);
    }
}

inline void usage() {
    std::cout << "Usage: bench_traversal [options]\n"
                 "Available options:\n"
                 "  -bvh     --bvh-file        Sets the BVH file to use\n"
                 "  -ray     --ray-file        Sets the ray file to use\n"
                 "           --tmin            Sets the minimum distance along the rays (default: 0)\n"
                 "           --tmax            Sets the maximum distance along the rays (default: 1e9)\n"
                 "           --bench           Sets the number of benchmark iterations (default: 1)\n"
                 "           --warmup          Sets the number of warmup iterations (default: 0)\n"
                 "  -gpu                       Runs the traversal on the GPU (disabled by default)\n"
                 "  -any                       Exits at the first intersection (disabled by default)\n"
                 "  -s       --single          Uses only single rays on the CPU (incompatible with --packet, disabled by default)\n"
                 "  -p       --packet          Uses only packets of rays on the CPU (incompatible with --single, disabled by default)\n"
                 "           --bvh-width       Sets the BVH width (4 or 8, default: 4)\n"
                 "           --ray-width       Sets the ray width (4 or 8, default: 8)\n"
                 "  -o       --output          Sets the output file name (no file is generated by default)\n";
}

static double bench_cpu_hybrid(Node8* nodes, Tri4* tris, Ray4* rays, Hit4* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_hybrid_ray4_bvh8_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_hybrid_ray4_bvh8_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_packet(Node8* nodes, Tri4* tris, Ray4* rays, Hit4* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_packet_ray4_bvh8_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_packet_ray4_bvh8_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_hybrid(Node8* nodes, Tri4* tris, Ray8* rays, Hit8* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_hybrid_ray8_bvh8_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_hybrid_ray8_bvh8_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_packet(Node8* nodes, Tri4* tris, Ray8* rays, Hit8* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_packet_ray8_bvh8_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_packet_ray8_bvh8_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_single(Node8* nodes, Tri4* tris, Ray1* rays, Hit1* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_single_ray1_bvh8_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_single_ray1_bvh8_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_hybrid(Node4* nodes, Tri4* tris, Ray4* rays, Hit4* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_hybrid_ray4_bvh4_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_hybrid_ray4_bvh4_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_packet(Node4* nodes, Tri4* tris, Ray4* rays, Hit4* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_packet_ray4_bvh4_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_packet_ray4_bvh4_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_hybrid(Node4* nodes, Tri4* tris, Ray8* rays, Hit8* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_hybrid_ray8_bvh4_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_hybrid_ray8_bvh4_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_packet(Node4* nodes, Tri4* tris, Ray8* rays, Hit8* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_packet_ray8_bvh4_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_packet_ray8_bvh4_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_cpu_single(Node4* nodes, Tri4* tris, Ray1* rays, Hit1* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_micro_time();
    if (any_hit) cpu_occluded_single_ray1_bvh4_tri4(nodes, tris, rays, hits, n);
    else         cpu_intersect_single_ray1_bvh4_tri4(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_micro_time();
    return (t1 - t0) / 1000.0;
}

static double bench_gpu(Node2* nodes, Tri1* tris, Ray1* rays, Hit1* hits, size_t n, bool any_hit) {
    auto t0 = anydsl_get_kernel_time();
    if (any_hit) nvvm_occluded_single_ray1_bvh2_tri1(nodes, tris, rays, hits, n);
    else         nvvm_intersect_single_ray1_bvh2_tri1(nodes, tris, rays, hits, n);
    auto t1 = anydsl_get_kernel_time();
    return (t1 - t0) / 1000.0;
}

int main(int argc, char** argv) {
    std::string ray_file;
    std::string bvh_file;
    std::string out_file;
    float tmin = 0.0f, tmax = 1e9f;
    int iters = 1;
    int warmup = 0;
    bool use_gpu = false;
    bool any_hit = false;
    int bvh_width = 4;
    int ray_width = 8;
    bool single = false, packet = false;

    for (int i = 1; i < argc; i++) {
        auto arg = argv[i];
        if (arg[0] == '-') {
            if (!strcmp(arg, "-h") || !strcmp(arg, "--help")) {
                usage();
                return 0;
            } else if (!strcmp(arg, "-bvh") || !strcmp(arg, "--bvh-file")) {
                check_argument(i, argc, argv);
                bvh_file = argv[++i];
            } else if (!strcmp(arg, "-ray") || !strcmp(arg, "--ray-file")) {
                check_argument(i, argc, argv);
                ray_file = argv[++i];
            } else if (!strcmp(arg, "--tmin")) {
                check_argument(i, argc, argv);
                tmin = strtof(argv[++i], nullptr);
            } else if (!strcmp(arg, "--tmax")) {
                check_argument(i, argc, argv);
                tmax = strtof(argv[++i], nullptr);
            } else if (!strcmp(arg, "--bench") || !strcmp(arg, "--bench-iters")) {
                check_argument(i, argc, argv);
                iters = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "--warmup") || !strcmp(arg, "--warmup-iters")) {
                check_argument(i, argc, argv);
                warmup = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "-gpu")) {
                use_gpu = true;
            } else if (!strcmp(arg, "-any")) {
                any_hit = true;
            } else if (!strcmp(arg, "-s") || !strcmp(arg, "--single")) {
                single = true;
            } else if (!strcmp(arg, "-p") || !strcmp(arg, "--packet")) {
                packet = true;
            } else if (!strcmp(arg, "--bvh-width")) {
                check_argument(i, argc, argv);
                bvh_width = strtol(argv[++i], nullptr, 10);
            }  else if (!strcmp(arg, "--ray-width")) {
                check_argument(i, argc, argv);
                ray_width = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "-o") || !strcmp(arg, "--output")) {
                check_argument(i, argc, argv);
                out_file = argv[++i];
            } else {
                std::cerr << "Unknown option '" << arg << "'" << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Invalid argument '" << arg << "'" << std::endl;
            return 1;
        }
    }

    if (bvh_file == "") {
        std::cerr << "No BVH file specified" << std::endl;
        return 1;
    }
    if (ray_file == "") {
        std::cerr << "No ray file specified" << std::endl;
        return 1;
    }

    if (use_gpu && single) {
        std::cerr << "Options '--gpu' and '--single' are incompatible" << std::endl;
        return 1;
    }
    if (single && packet) {
        std::cerr << "Options '--packet' and '--single' are incompatible" << std::endl;
        return 1;
    }
    if (bvh_width != 4 && bvh_width != 8) {
        std::cerr << "Invalid BVH width" << std::endl;
        return 1;
    }
    if (ray_width != 4 && ray_width != 8) {
        std::cerr << "Invalid ray width" << std::endl;
        return 1;
    }

    anydsl::Array<Node2> nodes2;
    anydsl::Array<Node4> nodes4;
    anydsl::Array<Node8> nodes8;
    anydsl::Array<Tri1>  tris1;
    anydsl::Array<Tri4>  tris4;

    if (use_gpu) {
        if (!load_bvh(bvh_file, nodes2, tris1, BvhType::BVH2_TRI1, true)) {
            std::cerr << "Cannot load BVH file" << std::endl;
            return 1;
        }
    } else if (bvh_width == 4) {
        if (!load_bvh(bvh_file, nodes4, tris4, BvhType::BVH4_TRI4, false)) {
            std::cerr << "Cannot load BVH file" << std::endl;
            return 1;
        }
    } else {
        if (!load_bvh(bvh_file, nodes8, tris4, BvhType::BVH8_TRI4, false)) {
            std::cerr << "Cannot load BVH file" << std::endl;
            return 1;
        }
    }

    anydsl::Array<Ray1> rays1;
    anydsl::Array<Ray4> rays4;
    anydsl::Array<Ray8> rays8;
    size_t ray_count = 0;
    if (use_gpu || single) {
        if (!load_rays(ray_file, rays1, tmin, tmax, use_gpu)) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        ray_count = rays1.size();
    } else if (ray_width == 4) {
        if (!load_rays(ray_file, rays4, tmin, tmax, false)) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        ray_count = rays4.size() * 4;
    } else {
        if (!load_rays(ray_file, rays8, tmin, tmax, false)) {
            std::cerr << "Cannot load rays" << std::endl;
            return 1;
        }
        ray_count = rays8.size() * 8;
    }
    
    std::cout << ray_count << " ray(s) in the distribution file." << std::endl;

    anydsl::Array<Hit1> hits1;
    anydsl::Array<Hit4> hits4;
    anydsl::Array<Hit8> hits8;
    if (use_gpu || single) {
        hits1 = std::move(anydsl::Array<Hit1>(use_gpu ? anydsl::Platform::Cuda : anydsl::Platform::Host, anydsl::Device(0), rays1.size()));
    } else if (ray_width == 4) {
        hits4 = std::move(anydsl::Array<Hit4>(rays4.size()));
    } else {
        hits8 = std::move(anydsl::Array<Hit8>(rays8.size()));
    }

    std::function<double()> bench;
    if (use_gpu) bench = [&] { return bench_gpu(nodes2.data(), tris1.data(), rays1.data(), hits1.data(), ray_count, any_hit); };
    else if (bvh_width == 4) {
        if (single) bench = [&] { return bench_cpu_single(nodes4.data(), tris4.data(), rays1.data(), hits1.data(), rays1.size(), any_hit); };
        else if (packet) {
            if (ray_width == 4) bench = [&] { return bench_cpu_packet(nodes4.data(), tris4.data(), rays4.data(), hits4.data(), rays4.size(), any_hit); };
            else                bench = [&] { return bench_cpu_packet(nodes4.data(), tris4.data(), rays8.data(), hits8.data(), rays8.size(), any_hit); };
        } else {
            if (ray_width == 4) bench = [&] { return bench_cpu_hybrid(nodes4.data(), tris4.data(), rays4.data(), hits4.data(), rays4.size(), any_hit); };
            else                bench = [&] { return bench_cpu_hybrid(nodes4.data(), tris4.data(), rays8.data(), hits8.data(), rays8.size(), any_hit); };
        }
    } else {
        if (single)      bench = [&] { return bench_cpu_single(nodes8.data(), tris4.data(), rays1.data(), hits1.data(), rays1.size(), any_hit); };
        else if (packet) {
            if (ray_width == 4) bench = [&] { return bench_cpu_packet(nodes8.data(), tris4.data(), rays4.data(), hits4.data(), rays4.size(), any_hit); };
            else                bench = [&] { return bench_cpu_packet(nodes8.data(), tris4.data(), rays8.data(), hits8.data(), rays8.size(), any_hit); };
        } else {
            if (ray_width == 4) bench = [&] { return bench_cpu_hybrid(nodes8.data(), tris4.data(), rays4.data(), hits4.data(), rays4.size(), any_hit); };
            else                bench = [&] { return bench_cpu_hybrid(nodes8.data(), tris4.data(), rays8.data(), hits8.data(), rays8.size(), any_hit); };
        }
    }

    for (int i = 0; i < warmup; i++) bench();

    std::vector<double> timings;
    for (int i = 0; i < iters; i++) {
        timings.push_back(bench());
    }

    size_t intr = 0;
    if (use_gpu) {
        anydsl::Array<Hit1> host_hits(hits1.size());
        anydsl::copy(hits1, host_hits);
        for (auto& hit : host_hits)
            intr += hit.tri_id >= 0;
        if (out_file != "") {
            std::ofstream of(out_file, std::ofstream::binary);
            for (auto& hit : host_hits)
                of.write((char*)&hit.t, sizeof(float));
        }
    } else if (single) {
        for (auto& hit : hits1)
            intr += hit.tri_id >= 0;
        if (out_file != "") {
            std::ofstream of(out_file, std::ofstream::binary);
            for (auto& hit : hits1)
                of.write((char*)&hit.t, sizeof(float));
        }
    } else if (ray_width == 4) {
        for (auto& hit : hits4) {
            for (int i = 0; i < 4; i++)
                intr += hit.tri_id[i] >= 0;
        }
        if (out_file != "") {
            std::ofstream of(out_file, std::ofstream::binary);
            for (auto& hit : hits4) {
                for (int i = 0; i < 4; i++)
                    of.write((char*)&hit.t[i], sizeof(float));
            }
        }
    } else {
        for (auto& hit : hits8) {
            for (int i = 0; i < 8; i++)
                intr += hit.tri_id[i] >= 0;
        }
        if (out_file != "") {
            std::ofstream of(out_file, std::ofstream::binary);
            for (auto& hit : hits8) {
                for (int i = 0; i < 8; i++)
                    of.write((char*)&hit.t[i], sizeof(float));
            }
        }
    }

    std::sort(timings.begin(), timings.end());
    auto sum = std::accumulate(timings.begin(), timings.end(), 0.0);
    auto avg = sum / timings.size();
    auto med = timings[timings.size() / 2];
    auto min = *std::min_element(timings.begin(), timings.end());
    std::cout << sum << "ms for " << iters << " iteration(s)" << std::endl;
    std::cout << ray_count * iters / (1000.0 * sum) << " Mrays/sec" << std::endl;
    std::cout << "# Average: " << avg << " ms" << std::endl;
    std::cout << "# Median: " << med  << " ms" << std::endl;
    std::cout << "# Min: " << min << " ms" << std::endl;
    std::cout << intr << " intersection(s)" << std::endl;
    return 0;
}
