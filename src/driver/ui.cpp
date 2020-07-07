#include "ui.h"

#include <SDL.h>
#include "imgui.h"
#include "imgui_sdl.h"

#include "interface.h"

#include "runtime/common.h"
#include "runtime/image.h"
#include "runtime/color.h"

#define CULL_BAD_COLOR
#define CATCH_BAD_COLOR
#define USE_MEDIAN_FOR_LUMINANCE_ESTIMATION

static SDL_Window* sWindow;
static SDL_Renderer* sRenderer;
static SDL_Texture* sTexture;
static std::vector<uint32_t> sBuffer;
static int sWidth;
static int sHeight;

// Interface
float* get_pixels();

static bool handle_events(uint32_t& iter, Camera& cam) {
    static bool camera_on = false;
    static bool arrows[4] = { false, false, false, false };
    static bool speed[2] = { false, false };
    const float rspeed = 0.005f;
    static float tspeed = 0.1f;

    SDL_Event event;
	int wheel = 0;
    bool hover = ImGui::IsAnyItemHovered() || ImGui::IsAnyWindowHovered();
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
                if (event.button.button == SDL_BUTTON_LEFT && !hover) {
                    SDL_SetRelativeMouseMode(SDL_TRUE);
                    camera_on = true;
                }
                break;
            case SDL_MOUSEBUTTONUP:
                if (event.button.button == SDL_BUTTON_LEFT && !hover) {
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                    camera_on = false;
                }
                break;
            case SDL_MOUSEMOTION:
                if (camera_on && !hover) {
                    cam.rotate(event.motion.xrel * rspeed, event.motion.yrel * rspeed);
                    iter = 0;
                }
                break;
            case SDL_MOUSEWHEEL:
				wheel = event.wheel.y;
                break;
            case SDL_QUIT:
                return true;
            default:
                break;
        }
    }

    int mouseX, mouseY;
    const int buttons = SDL_GetMouseState(&mouseX, &mouseY);

    // Setup low-level inputs (e.g. on Win32, GetKeyboardState(), or write to those fields from your Windows message loop handlers, etc.)
    ImGuiIO& io = ImGui::GetIO();
    io.DeltaTime = 1.0f / 60.0f;
    io.MousePos = ImVec2(static_cast<float>(mouseX), static_cast<float>(mouseY));
    io.MouseDown[0] = buttons & SDL_BUTTON(SDL_BUTTON_LEFT);
    io.MouseDown[1] = buttons & SDL_BUTTON(SDL_BUTTON_RIGHT);
    io.MouseWheel = static_cast<float>(wheel);

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

void rodent_ui_init(int width, int height) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
        error("Cannot initialize SDL.");

    sWidth  = width;
    sHeight = height;
    sWindow = SDL_CreateWindow(
        "Rodent",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        width,
        height,
        0);
    if (!sWindow)
        error("Cannot create window.");

    sRenderer = SDL_CreateRenderer(sWindow, -1, 0);
    if (!sRenderer)
        error("Cannot create renderer.");

    sTexture = SDL_CreateTexture(sRenderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, width, height);
    if (!sTexture)
        error("Cannot create texture");

    sBuffer.resize(width*height);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGuiSDL::Initialize(sRenderer, width, height);
}

void rodent_ui_close() {
    ImGuiSDL::Deinitialize();

    SDL_DestroyTexture(sTexture);
    SDL_DestroyRenderer(sRenderer);
    SDL_DestroyWindow(sWindow);
    SDL_Quit();

    sBuffer.clear();
}

void rodent_ui_settitle(const char* str) {
    SDL_SetWindowTitle(sWindow, str);
}

bool rodent_ui_handleinput(uint32_t& iter, Camera& cam) {
    return handle_events(iter, cam);
}

static void handle_imgui(uint32_t iter) {
    ImGui::Begin("Stats");
    ImGui::Text("Iter %i", iter);
    ImGui::Text("SPP %i", iter*get_spp());
    ImGui::End();

    float f;
    ImGui::Begin("ToneMapping");
    ImGui::SliderFloat("Exposure", &f, 0.0f, 1.0f);
    ImGui::End();
}

void rodent_ui_update(uint32_t iter) {
    update_texture(sBuffer.data(), sTexture, sWidth, sHeight, iter);
    SDL_RenderClear(sRenderer);
    SDL_RenderCopy(sRenderer, sTexture, nullptr, nullptr);

	ImGui::NewFrame();
    handle_imgui(iter);
    ImGui::Render();

    ImGuiSDL::Render(ImGui::GetDrawData());

    SDL_RenderPresent(sRenderer);
}