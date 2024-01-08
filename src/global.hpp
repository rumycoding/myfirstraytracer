#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <thread>
#include <assert.h>
#include "Vector.hpp"

#undef M_PI
#define M_PI 3.141592653589793f

extern const float  EPSILON;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline  bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ?
                  -0.5 * (b + sqrt(discr)) :
                  -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) std::swap(x0, x1);
    return true;
}

// enhance with thread safe generator
// https://www.codenong.com/21237905/
inline float get_random_float()
{
    static thread_local std::mt19937* generator = nullptr;
    static thread_local std::random_device* dev = nullptr;
    if (!generator){
        dev = new std::random_device();
        generator = new std::mt19937((*dev)());
    }
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

    return dist(*generator);
}

inline float get_random_float(float min, float max) {
    float r = get_random_float();

    return min + (max - min) * r;
}

inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};

inline float abs_cos_theta(Vector3f a, Vector3f b) {
    float result = clamp(-1, 1, dotProduct(a, b));
    if (result < 0) result = -result;

    return result;
}

inline float abs_float(float v) {
    if (v < 0) return -v;
    return v;
}

// 异号
inline bool unlike_signed(float a, float b) {
    return (a>0 && b<0) || (a<0 && b>0);
}