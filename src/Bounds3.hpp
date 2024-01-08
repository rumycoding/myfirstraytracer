//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>
#include "global.hpp"

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray) const;
    
    inline bool IntersectP(const Ray& ray, float &tenter, float &texit) const;

    Vector3f Sample(float &pdf) {
        pdf = 1.0f / (pMax.x - pMin.x) / (pMax.y - pMin.y) / (pMin.z - pMax.z);
        
        return Vector3f(get_random_float(pMin.x, pMax.x), get_random_float(pMin.y, pMax.y), get_random_float(pMin.z, pMax.z));
    }
};

inline bool Bounds3::IntersectP(const Ray& ray, float &tenter, float &texit) const {

    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    std::array<int, 3> dirIsNeg = {int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0)};
    auto invDir = ray.direction_inv;

    // x
    float txmin = (pMin.x - ray.origin.x) * invDir.x;
    float txmax = (pMax.x - ray.origin.x) * invDir.x;

    if (dirIsNeg[0] == 0) std::swap(txmin, txmax);

    // y
    float tymin = (pMin.y - ray.origin.y) * invDir.y;
    float tymax = (pMax.y - ray.origin.y) * invDir.y;

    if (dirIsNeg[1] == 0) std::swap(tymin, tymax);

    // z
    float tzmin = (pMin.z - ray.origin.z) * invDir.z;
    float tzmax = (pMax.z - ray.origin.z) * invDir.z;

    if (dirIsNeg[2] == 0) std::swap(tzmin, tzmax);

    tenter = std::max({txmin, tymin, tzmin});
    texit = std::min({txmax, tymax, tzmax});

    // the bounding box might be vertical, so tenter = texit
    return tenter <= texit && texit > 0;
}

inline bool Bounds3::IntersectP(const Ray& ray) const
{
    float _tenter, _texit;
    return IntersectP(ray, _tenter, _texit);
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
