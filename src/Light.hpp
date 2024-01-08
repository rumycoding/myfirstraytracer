//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include "Vector.hpp"
#include "Intersection.hpp"

class Light
{
public:
    virtual Vector3f Le() = 0;
    virtual void SamplePoint(Intersection &pos, float &pdf) const = 0;
    virtual Vector3f SampleDirection(Intersection &pos,float &pdf) const = 0;
    virtual Vector3f Power() = 0;
};
