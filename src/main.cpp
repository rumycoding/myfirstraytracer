#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include "AreaLight.hpp"
#include "Heart.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    bool usePhotonMap = true;

    // Change the definition here to change resolution
    Scene scene(784, 784);
    scene.usePhotonMap = usePhotonMap;

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);

    Material* pink = new Material(DIFFUSE, Vector3f(0.0f));
    pink->Kd = Vector3f(0.94f, 0.5f, 0.5f);
    Material* blue = new Material(DIFFUSE, Vector3f(0.0f));
    blue->Kd = Vector3f(0.25f, 0.87f, 0.81f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(1.0f, 0.94f, 0.96f);

    // heart model color
    // left wall
    Material* pink_heart = new Material(DIFFUSE, Vector3f(0.0f));
    pink_heart->Kd = Vector3f(0.99f, 0.576f, 0.686f);
    // yellow = 0.99f, 0.847f, 0.302f
    // light pink = 0.988,0.788,0.843

    // right wall
    Material* green_heart = new Material(DIFFUSE, Vector3f(0.0f));
    green_heart->Kd = Vector3f(0.59f, 0.773f, 0.43f);
    Material* glass_heart = new Material(FRESNELSPECULAR, Vector3f(0.0f));
    // glass
    glass_heart->ior = 1.5;
    glass_heart->roughness = 0.0f;
    // yellow_heart
    glass_heart->R = Vector3f(0.99f, 0.847f, 0.302f);
    glass_heart->T = Vector3f(0.99f, 0.847f, 0.302f);
    

    // light
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.8f);

    // microfacet material
    Material* cook = new Material(COOKTORRANCE, Vector3f(0.0f));
    cook->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    cook->ior = 20.0f;
    cook->roughness = 0.4f;
    Material* cook2 = new Material(COOKTORRANCE, Vector3f(0.0f));
    cook2->ior = 1.8f;
    cook2->roughness = 0.8f;
    cook2->Kd = Vector3f(0.725f, 0.71f, 0.68f);

    Material* blueglass = new Material(FRESNELSPECULAR, Vector3f(0.0f));
    // glass
    blueglass->ior = 1.5;
    blueglass->roughness = 0.0f;
    // blue glass
    blueglass->R = Vector3f(0.69f, 0.88f, 0.9f);
    blueglass->T = Vector3f(0.69f, 0.88f, 0.9f);


    Material* pinkglass = new Material(FRESNELSPECULAR, Vector3f(0.0f));
    // glass
    pinkglass->ior = 1.5;
    pinkglass->roughness = 0.0f;
    // pink glass
    pinkglass->R = Vector3f(1.0f, 0.94f, 0.96f);
    pinkglass->T = Vector3f(1.0f, 0.94f, 0.96f);

    // mirror example
    Material* mirror = new Material(MIRROR, Vector3f(0.0f));
    // mirror
    mirror->ior = 40;
    mirror->R = Vector3f(1.0f);
    mirror->T = Vector3f(1.0f);


    MeshTriangle floor("../models/cornellbox/floor.obj", white);
    // MeshTriangle shortbox("../models/cornellbox/shortbox.obj", translucent);
    // MeshTriangle tallbox("../models/cornellbox/tallbox.obj", mirror);
    MeshTriangle left("../models/cornellbox/left.obj", pink_heart);
    MeshTriangle right("../models/cornellbox/right.obj", green_heart);
    MeshTriangle light_("../models/cornellbox/light.obj", light);


    
    Heart h(Vector3f(278, 250, 320), Vector3f(90, 90,90), pinkglass, Vector3f(1,0,0), Vector3f(0,0,1), Vector3f(0,-1,0));
    // Heart h(Vector3f(278, 250, 320), Vector3f(90, 90,90), pinkglass, Vector3f(cos(-30.0f/180.0f *M_PI),sin(-30.0f/180.0f *M_PI),0), Vector3f(0,0,1), Vector3f(sin(-30.0f/180.0f *M_PI),-cos(-30.0f/180.0f *M_PI),0));
    
    // pos = ( 横着，高度，深度 )
    Sphere sphere(Vector3f(175, 200 , 150), 160.0/2.0f, pinkglass);
    Sphere sphere2(Vector3f(375, 300 , 300), 200.0/2.0f, blueglass);

    scene.Add(&floor);
    // scene.Add(&sphere);
    // scene.Add(&shortbox);
    // scene.Add(&tallbox);
    // scene.Add(&sphere2);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    scene.Add(&h);

    std::unique_ptr<Light> arealight = std::make_unique<AreaLight>(&light_);
    scene.Add(std::move(arealight));


    scene.buildBVH();

    // photon scattering
    if (usePhotonMap) {
        scene.buildPhotonMaps();
    }

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}