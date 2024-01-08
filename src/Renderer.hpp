//
// Created by goksu on 2/25/20.
//

#pragma once
#include "Scene.hpp"
#include <fstream>

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};
