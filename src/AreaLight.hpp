//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once
#include "Triangle.hpp"
#include "Light.hpp"
#include "global.hpp"

class AreaLight : public Light
{
private:
    const MeshTriangle* t;
    
    Vector3f toWorld(const Vector3f &a, const Vector3f &N) const{
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    AreaLight(const MeshTriangle* t): t(t) {
    }

    Vector3f Le() {
        return t->m->getEmission();
    }

    Vector3f Power() {
        // L = d2phi/(d omega * dA * cos(theta))
        // if we do the integral, will get the belowing result
        return Le() * t->area * M_PI;
    }

    void SamplePoint(Intersection &pos, float &pdf) const {
        t->Sample(pos, pdf);
    }

     Vector3f SampleDirection(Intersection &pos,float &pdf) const {
        // cosine-weighted hemisphere sampling:
        // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#Cosine-WeightedHemisphereSampling
        float x_1 = get_random_float(), x_2 = get_random_float();

        float theta = acos(sqrt(1-x_1));
        float phi = 2 * M_PI * x_2;

        Vector3f result(sin(theta)*cos(phi),sin(theta)*sin(phi), cos(theta));

        pdf = cos(theta) * sin(theta) / M_PI;

        return toWorld(result, pos.normal);
     }
};
