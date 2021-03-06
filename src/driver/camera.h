#pragma once

#include "runtime/float3.h"

static constexpr float pi = 3.14159265359f;

struct Camera {
    float3 eye;
    float3 dir;
    float3 right;
    float3 up;
    float w, h;

    inline Camera(const float3& e, const float3& d, const float3& u, float fov, float ratio) {
        eye = e;
        dir = normalize(d);
        right = normalize(cross(dir, u));
        up = normalize(cross(right, dir));

        w = std::tan(fov * pi / 360.0f);
        h = w / ratio;
    }

    inline void rotate(float yaw, float pitch) {
        dir = ::rotate(dir, right,  -pitch);
        dir = ::rotate(dir, up,     -yaw);
        dir = normalize(dir);
        right = normalize(cross(dir, up));
        up = normalize(cross(right, dir));
    }

    inline void roll(float angle) {
        right = ::rotate(right, dir, angle);
        up = ::rotate(up, dir, angle);
    }

    inline void update_dir(const float3& ndir, const float3& nup) {
        dir = ndir;
        right = normalize(cross(dir, nup));
        up = normalize(cross(right, dir));
    }

    inline void move(float x, float y, float z) {
        eye += right * x + up * y + dir * z;
    }
};