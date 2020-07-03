#include <memory>
#include <sstream>
#include <algorithm>
#include <string>
#include <cstring>
#include <chrono>
#include <cmath>
#include <array>

#ifndef DISABLE_GUI
#include <SDL.h>
#endif

#define CULL_BAD_COLOR
#define CATCH_BAD_COLOR
#define USE_MEDIAN_FOR_LUMINANCE_ESTIMATION

#ifndef NDEBUG
#include <fenv.h>
#endif

#include "interface.h"
#include "runtime/float3.h"
#include "runtime/common.h"
#include "runtime/image.h"
#include "runtime/color.h"

#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64)
#include <x86intrin.h>
#endif

static constexpr float pi = 3.14159265359f;

struct Camera {
    float3 eye;
    float3 dir;
    float3 right;
    float3 up;
    float w, h;

    Camera(const float3& e, const float3& d, const float3& u, float fov, float ratio) {
        eye = e;
        dir = normalize(d);
        right = normalize(cross(dir, u));
        up = normalize(cross(right, dir));

        w = std::tan(fov * pi / 360.0f);
        h = w / ratio;
    }

    void rotate(float yaw, float pitch) {
        dir = ::rotate(dir, right,  -pitch);
        dir = ::rotate(dir, up,     -yaw);
        dir = normalize(dir);
        right = normalize(cross(dir, up));
        up = normalize(cross(right, dir));
    }

    void move(float x, float y, float z) {
        eye += right * x + up * y + dir * z;
    }
};

void setup_interface(size_t, size_t);
float* get_pixels();
void clear_pixels();
void cleanup_interface();

#ifndef DISABLE_GUI
static bool handle_events(uint32_t& iter, Camera& cam) {
    static bool camera_on = false;
    static bool arrows[4] = { false, false, false, false };
    static bool speed[2] = { false, false };
    const float rspeed = 0.005f;
    static float tspeed = 0.1f;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        bool key_down = event.type == SDL_KEYDOWN;
        switch (event.type) {
            case SDL_KEYUP:
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:   return true;
                    case SDLK_KP_PLUS:  speed[0] = key_down; break;
                    case SDLK_KP_MINUS: speed[1] = key_down; break;
                    case SDLK_UP:       arrows[0] = key_down; break;
                    case SDLK_w:        arrows[0] = key_down; break;
                    case SDLK_DOWN:     arrows[1] = key_down; break;
                    case SDLK_s:        arrows[1] = key_down; break;
                    case SDLK_LEFT:     arrows[2] = key_down; break;
                    case SDLK_a:        arrows[2] = key_down; break;
                    case SDLK_RIGHT:    arrows[3] = key_down; break;
                    case SDLK_d:        arrows[3] = key_down; break;
                    case SDLK_c:
                        info("Camera Eye: ", cam.eye.x, " ", cam.eye.y, " ", cam.eye.z );
                        info("Camera Dir: ", cam.dir.x, " ", cam.dir.y, " ", cam.dir.z);
                        info("Camera Up:  ", cam.up.x, " ", cam.up.y, " ", cam.up.z);
                        break;
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    SDL_SetRelativeMouseMode(SDL_TRUE);
                    camera_on = true;
                }
                break;
            case SDL_MOUSEBUTTONUP:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                    camera_on = false;
                }
                break;
            case SDL_MOUSEMOTION:
                if (camera_on) {
                    cam.rotate(event.motion.xrel * rspeed, event.motion.yrel * rspeed);
                    iter = 0;
                }
                break;
            case SDL_QUIT:
                return true;
            default:
                break;
        }
    }

    if (arrows[0]) cam.move(0, 0,  tspeed);
    if (arrows[1]) cam.move(0, 0, -tspeed);
    if (arrows[2]) cam.move(-tspeed, 0, 0);
    if (arrows[3]) cam.move( tspeed, 0, 0);
    if (arrows[0] | arrows[1] | arrows[2] | arrows[3]) iter = 0;
    if (speed[0]) tspeed *= 1.1f;
    if (speed[1]) tspeed *= 0.9f;
    return false;
}

#define RGB_C(r,g,b) (((r) << 16) | ((g) << 8) | (b))

static inline rgb xyz_to_srgb(const rgb& c) {
    return rgb(  3.2404542f*c.x - 1.5371385f*c.y - 0.4985314f*c.z, 
                -0.9692660f*c.x + 1.8760108f*c.y + 0.0415560f*c.z,
                 0.0556434f*c.x - 0.2040259f*c.y + 1.0572252f*c.z);
}

static inline rgb srgb_to_xyz(const rgb& c) {
    return rgb( 0.4124564f*c.x + 0.3575761f*c.y + 0.1804375f*c.z,
                0.2126729f*c.x + 0.7151522f*c.y + 0.0721750f*c.z,
                0.0193339f*c.x + 0.1191920f*c.y + 0.9503041f*c.z);
}

static inline rgb xyY_to_srgb(const rgb& c) {
    return c.y == 0 ? rgb(0,0,0) : xyz_to_srgb(rgb(c.x*c.z/c.y, c.z, (1-c.x-c.y)*c.z/c.y));
}

static inline rgb srgb_to_xyY(const rgb& c) {
    const auto s = srgb_to_xyz(c);
    const auto n = s.x+s.y+s.z;
    return (n == 0)?rgb(0,0,0):rgb(s.x/n, s.y/n, s.y);
}

static inline float reinhard_modified(float L) {
    constexpr float WhitePoint = 4.0f;
    return (L*(1.0f+L/(WhitePoint*WhitePoint)))/(1.0f+L);
}


static float estimateLuminance(size_t width, size_t height) {
    auto film = get_pixels();
    float max_luminance = 0.00001f;

#ifdef USE_MEDIAN_FOR_LUMINANCE_ESTIMATION
    constexpr size_t WINDOW_S = 3;
    constexpr size_t EDGE_S = WINDOW_S/2;
    std::array<float, WINDOW_S*WINDOW_S> window;

    for (size_t y = EDGE_S; y < height-EDGE_S; ++y) {
        for (size_t x = EDGE_S; x < width-EDGE_S; ++x) {

            size_t i = 0;
            for(size_t wy = 0; wy <WINDOW_S; ++wy) {
                for(size_t wx = 0; wx <WINDOW_S; ++wx) {
                    const auto ix = x + wx - EDGE_S;
                    const auto iy = y + wy - EDGE_S;

                    auto r = film[(iy * width + ix) * 3 + 0];
                    auto g = film[(iy * width + ix) * 3 + 1];
                    auto b = film[(iy * width + ix) * 3 + 2];

                    window[i] = srgb_to_xyY(rgb(r,g,b)).z;
                    ++i;
                }
            }

            std::sort(window.begin(), window.end());
            const auto L = window[window.size()/2];
            max_luminance = std::max(max_luminance, L);
        }
    }
#else
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            auto r = film[(y * width + x) * 3 + 0];
            auto g = film[(y * width + x) * 3 + 1];
            auto b = film[(y * width + x) * 3 + 2];

            const auto L = srgb_to_xyY(rgb(r,g,b)).z;
            max_luminance = std::max(max_luminance, L);
        }
    }
#endif

    return max_luminance;
}

static void update_texture(uint32_t* buf, SDL_Texture* texture, size_t width, size_t height, uint32_t iter) {
    auto film = get_pixels();
    auto inv_iter = 1.0f / iter;
    auto inv_gamma = 1.0f / 2.2f;
    
    float max_luminance = estimateLuminance(width, height);

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            auto r = film[(y * width + x) * 3 + 0];
            auto g = film[(y * width + x) * 3 + 1];
            auto b = film[(y * width + x) * 3 + 2];

            const auto xyY = srgb_to_xyY(rgb(r,g,b));
#ifdef CULL_BAD_COLOR
            if(std::isinf(xyY.z)) {
#ifdef CATCH_BAD_COLOR
                buf[y * width + x] = RGB_C(255, 0, 150);//Pink
#endif
                continue;
            } else if(std::isnan(xyY.z)) {
#ifdef CATCH_BAD_COLOR
                buf[y * width + x] = RGB_C(0, 255, 255);//Cyan
#endif
                continue;
            } else if(xyY.x < 0.0f || xyY.y < 0.0f || xyY.z < 0.0f) {
#ifdef CATCH_BAD_COLOR
                buf[y * width + x] = RGB_C(255, 255, 0);//Orange
#endif
                continue;
            }
#endif

            //const float L = xyY.z / (9.6f * avg_luminance);
            const float L = xyY.z / max_luminance;
            const auto c = xyY_to_srgb(rgb(xyY.x, xyY.y, reinhard_modified(L)));

            buf[y * width + x] =
                (uint32_t(clamp(std::pow(c.x, inv_gamma), 0.0f, 1.0f) * 255.0f) << 16) |
                (uint32_t(clamp(std::pow(c.y, inv_gamma), 0.0f, 1.0f) * 255.0f) << 8)  |
                 uint32_t(clamp(std::pow(c.z, inv_gamma), 0.0f, 1.0f) * 255.0f);
        }
    }
    SDL_UpdateTexture(texture, nullptr, buf, width * sizeof(uint32_t));
}
#endif

static void save_image(const std::string& out_file, size_t width, size_t height, uint32_t iter) {
    ImageRgba32 img;
    img.width = width;
    img.height = height;
    img.pixels.reset(new float[width * height * 4]);

    auto film = get_pixels();
    auto inv_iter = 1.0f / iter;
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            auto r = film[(y * width + x) * 3 + 0];
            auto g = film[(y * width + x) * 3 + 1];
            auto b = film[(y * width + x) * 3 + 2];

            img.pixels[4 * (y * width + x) + 0] = r * inv_iter;
            img.pixels[4 * (y * width + x) + 1] = g * inv_iter;
            img.pixels[4 * (y * width + x) + 2] = b * inv_iter;
            img.pixels[4 * (y * width + x) + 3] = 1.0f;
        }
    }

    if (!save_exr(out_file, img))
        error("Failed to save EXR file '", out_file, "'");
}

static inline void check_arg(int argc, char** argv, int arg, int n) {
    if (arg + n >= argc)
        error("Option '", argv[arg], "' expects ", n, " arguments, got ", argc - arg);
}

static inline void usage() {
    std::cout << "Usage: rodent [options]\n"
              << "Available options:\n"
              << "   --help              Shows this message\n"
              << "   --width  pixels     Sets the viewport horizontal dimension (in pixels)\n"
              << "   --height pixels     Sets the viewport vertical dimension (in pixels)\n"
              << "   --eye    x y z      Sets the position of the camera\n"
              << "   --dir    x y z      Sets the direction vector of the camera\n"
              << "   --up     x y z      Sets the up vector of the camera\n"
              << "   --fov    degrees    Sets the horizontal field of view (in degrees)\n"
              << "   --spp    spp        Enables benchmarking mode and sets the number of iterations based on the given spp\n"
              << "   --bench  iterations Enables benchmarking mode and sets the number of iterations\n"
              << "   --nimg   iterations Enables output extraction every n iterations\n"
              << "   -o       image.exr  Writes the output image to a file" << std::endl;
}

constexpr int SPP_PER_ITERATION = RODENT_SPP;

int main(int argc, char** argv) {
    std::string out_file;
    size_t bench_iter = 0;
    size_t nimg_iter = 0;
    size_t width  = 1080;
    size_t height = 720;
    float fov = 60.0f;
    float3 eye(0.0f), dir(0.0f, 0.0f, 1.0f), up(0.0f, 1.0f, 0.0f);

    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-') {
            if (!strcmp(argv[i], "--width")) {
                check_arg(argc, argv, i, 1);
                width = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--height")) {
                check_arg(argc, argv, i, 1);
                height = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--eye")) {
                check_arg(argc, argv, i, 3);
                eye.x = strtof(argv[++i], nullptr);
                eye.y = strtof(argv[++i], nullptr);
                eye.z = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--dir")) {
                check_arg(argc, argv, i, 3);
                dir.x = strtof(argv[++i], nullptr);
                dir.y = strtof(argv[++i], nullptr);
                dir.z = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--up")) {
                check_arg(argc, argv, i, 3);
                up.x = strtof(argv[++i], nullptr);
                up.y = strtof(argv[++i], nullptr);
                up.z = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--fov")) {
                check_arg(argc, argv, i, 1);
                fov = strtof(argv[++i], nullptr); 
            } else if (!strcmp(argv[i], "--nimg")) {
                check_arg(argc, argv, i, 1);
                nimg_iter = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--spp")) {
                check_arg(argc, argv, i, 1);
                bench_iter = (size_t)std::ceil(strtoul(argv[++i], nullptr, 10) / (float)SPP_PER_ITERATION);
            } else if (!strcmp(argv[i], "--bench")) {
                check_arg(argc, argv, i, 1);
                bench_iter = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "-o")) {
                check_arg(argc, argv, i, 1);
                out_file = argv[++i];
            } else if (!strcmp(argv[i], "--help")) {
                usage();
                return 0;
            } else {
                error("Unknown option '", argv[i], "'");
            }
            continue;
        }
        error("Unexpected argument '", argv[i], "'");
    }

    std::string iter_file_prefix = "iteration_";
    if(out_file != "")
        iter_file_prefix = FilePath(out_file).remove_extension() + "_";

    Camera cam(eye, dir, up, fov, (float)width / (float)height);

#ifdef DISABLE_GUI
    info("Running in console-only mode (compiled with -DDISABLE_GUI).");
    if (bench_iter == 0) {
        warn("Benchmark iterations not set. Defaulting to 1.");
        bench_iter = 1;
    }
#else
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        error("Cannot initialize SDL.");

    auto window = SDL_CreateWindow(
        "Rodent",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        width,
        height,
        0);
    if (!window)
        error("Cannot create window.");

    auto renderer = SDL_CreateRenderer(window, -1, 0);
    if (!renderer)
        error("Cannot create renderer.");

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, width, height);
    if (!texture)
        error("Cannot create texture");

    std::unique_ptr<uint32_t> buf(new uint32_t[width * height]);
#endif

    setup_interface(width, height);

    // Force flush to zero mode for denormals
#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64)
    _mm_setcsr(_mm_getcsr() | (_MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON));
#endif

#if !defined(NDEBUG)
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    auto spp = get_spp();
    bool done = false;
    uint64_t timing = 0;
    uint32_t frames = 0;
    uint32_t iter = 0;
    uint32_t niter = 0;
    std::vector<double> samples_sec;
    while (!done) {
#ifndef DISABLE_GUI
        done = handle_events(iter, cam);
#endif
        if (iter == 0)
            clear_pixels();

        Settings settings {
            Vec3 { cam.eye.x, cam.eye.y, cam.eye.z },
            Vec3 { cam.dir.x, cam.dir.y, cam.dir.z },
            Vec3 { cam.up.x, cam.up.y, cam.up.z },
            Vec3 { cam.right.x, cam.right.y, cam.right.z },
            cam.w,
            cam.h
        };

        auto ticks = std::chrono::high_resolution_clock::now();
        render(&settings, iter++);
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - ticks).count();

        if (bench_iter != 0) {
            samples_sec.emplace_back(1000.0 * double(spp * width * height) / double(elapsed_ms));
            if (samples_sec.size() == bench_iter)
                break;
        }

        if (nimg_iter != 0) {
            ++niter;
            if(nimg_iter <= niter) {
                niter = 0;
                std::stringstream sstream;
                sstream << iter_file_prefix << iter * spp << ".exr";
                save_image(sstream.str(), width, height, iter);
                info("Iteration image saved to '", sstream.str(), "'");
            }
        }

        frames++;
        timing += elapsed_ms;
        if (frames > 10 || timing >= 2500) {
            auto frames_sec = double(frames) * 1000.0 / double(timing);
#ifndef DISABLE_GUI
            std::ostringstream os;
            os << "Rodent [" << frames_sec << " FPS, "
               << iter * spp << " " << "sample" << (iter * spp > 1 ? "s" : "") << "]";
            SDL_SetWindowTitle(window, os.str().c_str());
#endif
            frames = 0;
            timing = 0;
        }

#ifndef DISABLE_GUI
        update_texture(buf.get(), texture, width, height, iter);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
#endif
    }

#ifndef DISABLE_GUI
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
#endif

    if (out_file != "") {
        save_image(out_file, width, height, iter);
        info("Image saved to '", out_file, "'");
    }

    cleanup_interface();

    if (bench_iter != 0) {
        auto inv = 1.0e-6;
        std::sort(samples_sec.begin(), samples_sec.end());
        info("# ", samples_sec.front() * inv,
             "/", samples_sec[samples_sec.size() / 2] * inv,
             "/", samples_sec.back() * inv,
             " (min/med/max Msamples/s)");
    }
    return 0;
}
