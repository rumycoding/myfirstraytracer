//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"

enum MaterialType { DIFFUSE, COOKTORRANCE, FRESNELSPECULAR, MIRROR};
enum TransportMode { 
    Radiance,   // from camera
    Importance  // from light, for photon map build
};
enum SampleType {Reflect, TransmitOrDiffuse, All};  // actually, this is useless lol

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // a is roughness 
    // https://learnopengl.com/PBR/Theory
    float normalDistributionFunction(const Vector3f &N, const Vector3f &H, float a)
    {
        float a2 = a*a;
        float nom = a2;
        float NdotH  = std::max(dotProduct(N, H), 0.0f);
        float NdotH2 = NdotH*NdotH;
        float denom = M_PI *pow((NdotH2 * (a2 - 1.0) + 1.0) ,2);

        return nom / denom;
    }

    // the smaller ndotV is，the more overshading, the smaller the term is
    float GeometrySchlickGGX(float NdotV, float k)
    {
        float nom   = NdotV;
        float denom = NdotV * (1.0 - k) + k;
        
        return nom / denom;
    }
    
    // k is remmaping of roughness
    // V: view direction
    // L: light direction
    float GeometrySmith(Vector3f N, Vector3f V, Vector3f L, float k)
    {
        float NdotV = std::max(dotProduct(N, V), 0.0f);
        float NdotL = std::max(dotProduct(N, L), 0.0f);
        float ggx1 = GeometrySchlickGGX(NdotV, k);
        float ggx2 = GeometrySchlickGGX(NdotL, k);
        
        return ggx1 * ggx2;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
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
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior = 20.0f;   //randomly pick a number
    float roughness = 0.4f;
    Vector3f Kd, Ks;
    Vector3f R, T;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v) const;
    inline Vector3f getEmission() const;
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N, SampleType t = SampleType::All);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, SampleType t = SampleType::All);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, bool isDirectLight = true, TransportMode m=TransportMode::Radiance, SampleType t = SampleType::All);
    // return portion of light reflected
    inline float RF(const Vector3f &I, const Vector3f &N);
    inline bool isDiffuse() {
        return (m_type != FRESNELSPECULAR) && (m_type != MIRROR);
    }

    // determined if the ray wo is reflecting or refracting
    bool isReflect(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
        float cosi = dotProduct(wi, N);
        float coso = dotProduct(wo, N);
        return cosi * coso >= 0; // cosi and coso has same direction
    }

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() const {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) const {
    return Vector3f();
}


// 给定入射方向和法线，出射方向是什么样的
// wi is pointing outwards of the insertion point
Vector3f Material::sample(const Vector3f &wi, const Vector3f &N, SampleType t){
    switch(m_type){
        case DIFFUSE:
        case COOKTORRANCE:
        {
            // TODO: cooktorrance model is problematic for photon mapping

            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            auto result = toWorld(localRay, N);
            return result;

            //  float x_1 = get_random_float(), x_2 = get_random_float();

            // float theta = acos(sqrt(1-x_1));
            // float phi = 2 * M_PI * x_2;

            // Vector3f result(sin(theta)*cos(phi),sin(theta)*sin(phi), cos(theta));

            // result = toWorld(result, N);
            // return result;
            break;
        }
        case FRESNELSPECULAR:
        {
            if (t == SampleType::Reflect) {
                return reflect(-wi, N);
            }

            if (t == SampleType::TransmitOrDiffuse) {
                return refract(-wi, N, ior);
            }

            float fresnel_term = 0;
            fresnel(-wi, N, ior, fresnel_term);
            float r = get_random_float();
            // float r = 0;

            if (r < fresnel_term) {
                // reflection
                return reflect(-wi, N);
            } else {
                return refract(-wi, N, ior);
            }
            break;
        }
        case MIRROR:
        {
            return reflect(-wi, N);
        }
    }

    return {};
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, SampleType t){
    switch(m_type){
        case DIFFUSE:
        case COOKTORRANCE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case FRESNELSPECULAR:
        {
            if (t == SampleType::Reflect || t == SampleType::TransmitOrDiffuse) {
                return 1.0f;
            }

            float fresnel_term = 0;
            fresnel(-wi, N, ior, fresnel_term);
            if (isReflect(wi, wo, N)) {
                return fresnel_term;
            } else {
                return (1-fresnel_term);
            }
        }
        case MIRROR:
        {
            if (t == SampleType::TransmitOrDiffuse) {
                return 0;
            }

            return 1;
        }
    }

    return {};
}

// calculating brdf
// wi and wo is pointing out of the point
// result: dL(wo)/dEi(wi) = dL(wo)/(Li(wi)cosi*dwi)
Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N, bool isDirectLight, TransportMode m, SampleType t){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                // 假设入射光和出射光radiance一致（课件上有公式）
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }

        case COOKTORRANCE:
        {
            Vector3f half_vector = normalize(wi + wo);
            float fresnel_term;

            fresnel(-wi, half_vector, ior, fresnel_term);
            // fresnel(wi, N, ior, fresnel_term);

            float normal_distribution_term = normalDistributionFunction(N, half_vector, roughness);

            float k = isDirectLight ? pow(roughness + 1.0, 2)/8.0 : pow(roughness, 2) / 2.0;

            // float geometry_term = GeometrySmith(N, wi, wo, pow(roughness, 2) / 2.0f);
            float geometry_term = GeometrySmith(N, wi, wo, k);


            float norminator = normal_distribution_term * geometry_term * fresnel_term;

            float denom = 4.0f * dotProduct(wo, N) * dotProduct(wi, N);

            Vector3f specular = norminator / std::max(denom, 0.001f);

            float ks_ = fresnel_term;   // reflected portion
            float kd_ = std::max(0.0f, 1.0f-ks_);

            if (t == SampleType::Reflect) {
                return specular;
            }

            if (t == SampleType::TransmitOrDiffuse) {
                return kd_ * Kd;
            }

            // refers to opengl PBR theory: https://learnopengl.com/PBR/Theory
            // in cook-torrance reflectance equation part. Should see that in specular term, ks_ already presents
            auto result = kd_ * Kd / M_PI + specular;

            return result;
        }

        case FRESNELSPECULAR:
        {
            float fresnel_term;
            fresnel(-wi, N, ior, fresnel_term);
            float abscosi = dotProduct(wi, N);
            if (abscosi < 0) abscosi = - abscosi;
            if (isReflect(wi, wo, N) || (t == SampleType::Reflect)) {
                if (m == TransportMode::Radiance) {
                    return R * fresnel_term / abscosi;
                }

                // during photon emitting steps, as we do not multiply abscosi when calculating power of emitting photons, so
                // we do not need to divide abscosi here!
                return R * fresnel_term;
            } else {
                float cosi = clamp(-1, 1, dotProduct(wi, N));
                float etai = 1, etat = ior;
                if (cosi < 0) std::swap(etai, etat);

                if (m == TransportMode::Radiance) {
                    return T*(1-fresnel_term) * etat * etat / (etai * etai) / abscosi;
                }

                return T*(1-fresnel_term);
            }
        }
        case MIRROR:
        {
            if (t == SampleType::TransmitOrDiffuse) {
                return 0;
            }

            float fresnel_term;
            fresnel(-wi, N, ior, fresnel_term);
            float abscosi = dotProduct(wi, N);
            if (abscosi < 0) abscosi = - abscosi;
            return fresnel_term * R/abscosi;
        }
    }

    return {};
}

// for insertion light: wi is pointing outwards of the point
// for leaving light: wi is pointing towards the intersection point
float Material::RF(const Vector3f &wi, const Vector3f &N) {
    switch(m_type) {
        case DIFFUSE:
            return 0;
        case COOKTORRANCE:
        case FRESNELSPECULAR:
        {
            float fresnel_term = 0;
            fresnel(-wi, N, ior, fresnel_term);
            return fresnel_term;
            break;
        }
        case MIRROR:
            return 1.0f;
    }
    return 0;
}

#endif //RAYTRACING_MATERIAL_H
