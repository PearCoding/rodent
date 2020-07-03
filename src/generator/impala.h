#pragma once

#include <sstream>
#include <cmath>
#include <algorithm>

inline std::string escape_f32(float f) {
    if(std::isinf(f) && !std::signbit(f)) {
        return "flt_inf";
    } else if(std::isinf(f) && std::signbit(f)) {
        return "-flt_inf";
    } else {
        std::stringstream sstream;
        sstream << f << "f";
        return sstream.str();
    }
}

inline std::string make_id(const std::string &str)
{
    auto id = str;
    std::transform(id.begin(), id.end(), id.begin(), [](char c) {
        if (std::isspace(c) || !std::isalnum(c))
            return '_';
        return c;
    });
    return id;
}

inline std::string fix_file(const std::string &str)
{
    auto id = str;
    std::transform(id.begin(), id.end(), id.begin(), [](char c) {
        if (c == '\\')
            return '/';
        return c;
    });
    return id;
}