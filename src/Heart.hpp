#pragma once
#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"
#include "global.hpp"

// some blogs for drawing heart
// https://www.cnblogs.com/WhyEngine/p/3885126.html

// some real heart => I mainly follow this
// https://www.shadertoy.com/view/XtXGR8
// https://www.shadertoy.com/view/XtVSRh
// http://mathworld.wolfram.com/HeartSurface.html

class Heart : public Object{
private:
    float t_step = 0.003;
    float init_step = 0.05;
    float accepted = 0.0000000000001;

    // range of x,y,z (manually)
    // x -> -1.3 1.3  => -1.5, 1.5
    // y -> -0.85 0.85 => -1, 1
    // z -> -1.2 1.4 => -1.4, 1.6
    // min_max[0] is min, min_max[1] is max!
    const std::vector<Vector3f> min_max = {Vector3f(-1.5, -1, -1.4), Vector3f(1.5, 1, 1.6)};
    Bounds3 boundInWorld;   // will remember it after initialization

    // (x^2+2.25y^2+z^2-1)^3=(x^2+0.1125y^2)*z^3
    float f(Vector3f p) const {
        Vector3f pp = p * p;
        Vector3f ppp = pp * p;
        float a = pp.x + 2.25 * pp.y + pp.z - 1.0;
        return a * a * a - (pp.x + 0.1125 * pp.y) * ppp.z;
    }

    // Bisection solver for t
    float bisect(Ray &r, float tstart, float tend) const {
        float leftf = f(r(tstart));
        float rightf = f(r(tend));
        float mid = (tstart + tend) /2.0f;
        float midf = 0;

        // assume locally monotone
        for (int i=0; i<30; i++) {
            midf = f(r(mid));
            if (std::fabs(midf) < accepted) return mid;

            if (unlike_signed(midf, leftf)) {
                tend = mid;
                rightf = midf;
            } else {
                tstart = mid;
                leftf = midf;
            }
            mid = (tstart + tend) /2.0f;
        }

        return mid;
    }

    // Analytical gradient
    // (-2 x z^3+6 x (-1.+x^2+2.25 y^2+z^2)^2) 
    // (-0.225 y z^3+13.5 y (-1.+x^2+2.25 y^2+z^2)^2)
    // (z (-3 x^2 z-0.3375 y^2 z+6 (-1.+x^2+2.25 y^2+z^2)^2))
    Vector3f normal(Vector3f p) const {
        Vector3f v = Vector3f(p.x, p.y, p.z);
        Vector3f vv = v * v;
        Vector3f vvv = vv * v;
        float a = -1.0 + dotProduct(vv, Vector3f(1, 2.25, 1));
        a *= a;
        
        return normalize(Vector3f(
            -2.0 * v.x * vvv.z +  6.0 * v.x * a,
            -0.225 * v.y * vvv.z + 13.5 * v.y * a,
            v.z * (-3.0 * vv.x * v.z - 0.3375 * vv.y * v.z + 6.0 * a)));
    }

    // might be problematic... if my math does it correctly.... hope so....
    Vector3f normalToWorld(Vector3f localp) const {
        Vector3f result(dotProduct(localp, rot_x), dotProduct(localp, rot_y), dotProduct(localp, rot_z));

        return normalize(result/scale);
    }

    // is_vector means it is a direction instead of the point
    // remmber in the homogeneous space, it means the last term is 0
    Vector3f toWorld(Vector3f localp, bool is_vector = false) const {
        Vector3f result(dotProduct(localp, rot_x), dotProduct(localp, rot_y), dotProduct(localp, rot_z));

        if (is_vector) return result * scale;

        return result * scale + center;
    }

    Vector3f toLocal(Vector3f worldp, bool is_vector= false) const {
        Vector3f result(worldp);
        if (!is_vector) result = result - center;

        result = result / scale; // divide is element-wise

        Vector3f rot_transpose_row_1 = Vector3f(rot_x.x, rot_y.x, rot_z.x);
        Vector3f rot_transpose_row_2 = Vector3f(rot_x.y, rot_y.y, rot_z.y);
        Vector3f rot_transpose_row_3 = Vector3f(rot_x.z, rot_y.z, rot_z.z);

        return Vector3f(dotProduct(result, rot_transpose_row_1), dotProduct(result, rot_transpose_row_2), dotProduct(result, rot_transpose_row_3));
    }

    // calculating bound of this heart (should be sligtly larger than the heart itself)
    void calculateBoundInWorld() {
        for (int i=0; i<2; i++) {
            for (int j=0; j<2; j++) {
                for (int k=0; k<2; k++) {
                    // convert bounds of local bound to world
                    // will get the world bound
                    boundInWorld = Union(boundInWorld, toWorld(Vector3f(min_max[i][0], min_max[j][1], min_max[k][2])));
                }
            }
        }
    }

    inline Bounds3 localBound() const {
        return Bounds3(min_max[0], min_max[1]);
    }

public:
    Vector3f center;
    Vector3f scale;
    Material *m;
    Vector3f rot_x; // row of the rotation matrix, row1
    Vector3f rot_y; // row 2
    Vector3f rot_z; // row 3


    Heart(const Vector3f &c, const Vector3f &s=Vector3f(1), Material* mt = new Material(), const Vector3f  rotx=Vector3f(1,0,0), const Vector3f roty=Vector3f(0,1,0), const Vector3f rotz=Vector3f(0,0,1)): center(c), scale(s), m(mt), rot_x(rotx), rot_y(roty), rot_z(rotz) {
        // initialize world bound
        calculateBoundInWorld();
    }

    bool intersect(const Ray& ray) {
        Intersection intsec = getIntersection(ray);
        return intsec.happened;
    }


    bool intersect(const Ray& ray, float &tnear, uint32_t &index) {
        Intersection intsec = getIntersection(ray);
        tnear = intsec.distance;
        return intsec.happened;
    }

    Intersection getIntersection(Ray ray) {
        Intersection result;
        result.happened = false;
        Ray local_ray = Ray(toLocal(ray.origin), normalize(toLocal(ray.direction, true)));
        Bounds3 local_bound = localBound();

        float tenter, texit;

        // quick test with local bound
        bool has_intersect = local_bound.IntersectP(local_ray, tenter, texit);
        if (!has_intersect) return result;

        tenter = std::max({0.0f, tenter});

        float value = f(local_ray(tenter+init_step-t_step));

        float tfind = -1;

        // !!! be careful with the case that the ray itself starts from a point on the surface
        // raymarching for finding intersection
        for (float t = tenter + init_step; t<=texit; t+= t_step) {
            float new_value = f(local_ray(t));

            if (unlike_signed(value, new_value)) {
                // bisect ray to find t find
                tfind = bisect(local_ray, t - t_step, t);
                break;
            }

            value = new_value;
        }

        if (tfind < 0) return result;

        Vector3f localp = local_ray(tfind);

        // convert intersection to world
        result.happened = true;
        result.coords = toWorld(Vector3f(localp));
        // if the ray origin inside the heart, should put the normal to the inverse direction
        result.normal = normalToWorld(normal(localp));
        result.m = this->m;
        result.obj = this;
        result.distance = toWorld(tfind * local_ray.direction, true).norm();

        return result;
    }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const
    { N = normal(P); }

    Vector3f evalDiffuseColor(const Vector2f &st)const {
        // return m->getColor();
        return {};
    }

    Bounds3 getBounds(){
        return boundInWorld;
    }

    // actually, the pdf will not be used if the object is not a light
    // I will not bother writing the pdf right...
    // ATTENTION: pdf inaccurate
    void Sample(Intersection &pos, float &pdf) const{
        pos.coords = toWorld(localBound().Sample(pdf));
        return;
    }

    // I will not consider calculating the surface area of it...
    // ATTENTION: area inaccurate
    float getArea(){
        return boundInWorld.SurfaceArea();
    }
    bool hasEmit(){
        return m->hasEmission();
    }
    bool isDiffuse() {
        return m->isDiffuse();
    }
};