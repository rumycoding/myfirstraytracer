//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float power_sum = 0;
    for (uint32_t i=0; i<lights.size(); i++) {
        power_sum += lights[i]->Power().norm();
    }

    float p = get_random_float() * power_sum;
    power_sum = 0;
    for (uint32_t k = 0; k < lights.size(); ++k) {
        power_sum += lights[k]->Power().norm();
        if (p <= power_sum) {
            lights[k]->SamplePoint(pos, pdf);
            break;
        }
    }
}

// randomly sample a caustics object, 
// powScale in the probability a random ray will hit certain caustic object
// it used the SAH heuristics
void Scene::sampleCaustics(Intersection &pos, float &powScale) const {
    float surface_area_sum = 0;

    for (int i=0; i<objects.size(); i++) {
        if (!objects[i]->isDiffuse()) {
            surface_area_sum += objects[i] -> getBounds().SurfaceArea();
        }
    }

    powScale = surface_area_sum / bvh->WorldBound().SurfaceArea();

    float p = get_random_float() * surface_area_sum;
    float pdf;
    surface_area_sum = 0;
    for (int i=0; i<objects.size(); i++) {
        if (!objects[i]->isDiffuse()) {
            surface_area_sum += objects[i]->getBounds().SurfaceArea();
            if (p <= surface_area_sum) {
                objects[i]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    auto p = intersect(ray);

    if (!p.happened) return Vector3f(0);
    if (p.m->hasEmission()) return p.m->getEmission();

    Vector3f L_dir = Vector3f(0);
    auto N = p.normal.normalized();
    if (p.m->isDiffuse()) {
        // calculate direct light
        Intersection pos_light;
        float pdf_light;
        sampleLight(pos_light, pdf_light);

        // test light to point is blocked by any other objects
        auto point_to_light_dir = normalize(pos_light.coords - p.coords);
        // move the coord slightly out for intersection test
        auto testlightblock_outray_origin = p.coords + ((dotProduct(point_to_light_dir, N) >0) ? N * EPSILON : -N * EPSILON);
        auto point_to_light_intersect = intersect(Ray(testlightblock_outray_origin, point_to_light_dir));
        
        auto emit = pos_light.emit;
        auto NN = pos_light.normal.normalized();
        // wi and wo should be pointing out of the point
        auto f_brdf_light = p.m->eval(point_to_light_dir, -ray.direction, N, true, TransportMode::Radiance);
        // incoming light projected to normal direction
        auto cos_theta = clamp(0, 1, dotProduct(point_to_light_dir, N));
        // incoming light normal direction projected to incoming light direction (多少光是照着物体的)
        auto cos_theta_x = clamp(0,1, dotProduct(NN, -point_to_light_dir));
        auto light_to_point_vector = p.coords - pos_light.coords;
        auto light_to_point_distance = (p.coords - pos_light.coords).norm();
        // if not blocked, then light to point intersect is point itself
        // light to point intersection not happened, or happened but is the light distance
        if (!point_to_light_intersect.happened || 
        (( point_to_light_intersect.distance - light_to_point_distance) > -0.001)) {
            // emit * f_brdf_light is 每一项相乘
            L_dir = emit * f_brdf_light * cos_theta * cos_theta_x / pdf_light / std::pow(light_to_point_distance, 2);
        }
    }


    // calculate indirect light
    Vector3f L_indir = Vector3f(0);

    if (!usePhotonMap || (!p.m->isDiffuse())) {
        // old path tracing code here... 
        auto p_RR = get_random_float();
        if (p_RR < RussianRoulette) {
            // passed russian roulette test
            auto wo = p.m -> sample(-ray.direction, N);
            auto cosi = dotProduct(-ray.direction, N);
            auto coso = dotProduct(wo, N);
            auto isReflect = p.m->isReflect(-ray.direction, wo, N);
            auto pdf_hemi = p.m -> pdf(-ray.direction, wo, N);

            if (pdf_hemi > 0) {
                // as the origin of the reflection light, as it has a small offset, when test intersection with 
                // objects in the scene, we will not intersect with the point p
                auto outray_origin = p.coords + (dotProduct(wo, N) > 0 ? EPSILON * N : -EPSILON * N);

                auto outray = Ray(outray_origin, wo);

                // check if outray hit non-emitting object
                auto outray_intersection = intersect(outray);

                // if material is FRESNELSPECULAR, even the hitted object
                // has emission, it should be counted
                if (outray_intersection.happened && (!p.m->isDiffuse() || !outray_intersection.m->hasEmission())) {
                    auto emit = castRay(Ray(outray_origin, wo), depth + 1);
                    auto f_brdf_light = p.m->eval(wo, -ray.direction, N, false, TransportMode::Radiance);

                    L_indir = emit * f_brdf_light * abs_cos_theta(wo, N) / pdf_hemi / RussianRoulette;
                }
            }
        }
    } else {
        // use photon map && hits a diffuse object
        // use caustics photon map to evaluate caustics effect
        L_indir = causticPM.radianceEstimate(p.coords, N, -ray.direction, p.m);

        // use global photon map to evaluate indirect light
        auto wo = p.m -> sample(-ray.direction, N);
        auto pdf_hemi = p.m -> pdf(-ray.direction, wo, N);

        auto outray_origin = p.coords + (dotProduct(wo, N) > 0 ? EPSILON * N : -EPSILON * N);

        auto outray = Ray(outray_origin, wo);

        // check if outray hit diffuse object
        auto outray_intersection = intersect(outray);

        if (outray_intersection.happened && (!outray_intersection.m->hasEmission()) && outray_intersection.m->isDiffuse()) {
            // if the outray hits a diffuse object

            // estimate the radiance exiting the other diffuse object
            auto emit = globalPM.radianceEstimate(outray_intersection.coords, outray_intersection.normal.normalized(), -outray.direction, outray_intersection.m);

            auto f_brdf = p.m->eval(wo, -ray.direction, N, false, TransportMode::Radiance);


            L_indir = L_indir + emit * f_brdf * abs_cos_theta(wo, N) / pdf_hemi;
        }
    }
    
    return L_dir + L_indir;
}

void Scene::buildPhotonMaps() {
    buildPhotonMap(globalPM, globalPhotons);
    std::cout<< "  containing " << globalPM.size() << std::endl;
    
    // should not scale with 0.5, but it looks good....
    buildPhotonMap(causticPM, causticPhotons, 0.5);
    std::cout<< "  containing " << causticPM.size() << std::endl;
}

void Scene::buildPhotonMap(PhotonMap &pm, int numPhotons, float scale) {
    printf(" - Generating photon map %i...\n\n", pm.type);
    // for simplicity, assume that we only have a single light source
    assert(lights.size() == 1);

    for (int i=0; i<numPhotons; i++) {
        // sample light source
        Intersection pos_light;
        float pdf_light;
        sampleLight(pos_light, pdf_light);
        int light_index = 0;

        // init power with light source power
        Vector3f power = lights[light_index] -> Power();

        // based on the assumption that we will have a single light source
        // so we can simply multiply the power with 1/numPhotons
        Vector3f power_scale = Vector3f(scale / (float)numPhotons);
        Vector3f dir;

        if (pm.type != CAUSTIC) {
            // random sample a direction
            float pdf_dir = 0;
            dir = lights[light_index]->SampleDirection(pos_light, pdf_dir);
        } else {
            // scale power by SAH, as we are only shooting rays to parts of the scene
            // this is only served as heurstics for estimating the power towards caustics object
            // note here I did not consider situation like the caustic object is blocked by other objects in the scene
            float power_scale_caustics = 0;

            Intersection pos_caustics;
            // random sample a caustics object
            // random sample a point on the caustic surface
            sampleCaustics(pos_caustics, power_scale_caustics);

            power_scale = power_scale * power_scale_caustics;

            dir = (pos_caustics.coords - pos_light.coords).normalized();
        }

        // set init ray
        Ray ray(pos_light.coords, dir);

        // scale light power (if the ray is parallel to light source normal, it will shoot more power)
        // I think no need to scale here, as the sampling function we used is already cosine weighted
        // but... main reason for removing this line is that the figure looks better!!!
        // power_scale = power_scale * abs_cos_theta(dir, pos_light.normal.normalized());

        int depth = 0;
        bool hasSpecular = false;

        while (depth <= maxDepth) {
            depth ++;
            // cast ray
            Intersection ray_intsec = intersect(ray);
            Vector3f N = ray_intsec.normal.normalized();

            if (!ray_intsec.happened) {
                // the ray is casted to the sky!
                break;
            }

            if (ray_intsec.obj->isDiffuse()) {
                if (pm.type == CAUSTIC) {
                    if (hasSpecular) {
                        // store photon to the map
                        pm.store(ray_intsec.coords, ray.direction, power * power_scale);
                    }

                    break;
                } else {
                    // store photon to the map
                    pm.store(ray_intsec.coords, ray.direction, power * power_scale);
                }
            } else {
                // hits a specular object
                hasSpecular = true;
            }

            // generate next ray
            auto p_RR = get_random_float();
            if (p_RR < RussianRoulette) {
                // sample next ray
                auto wo = ray_intsec.m -> sample(-ray.direction, N);
                float pdf_wo = ray_intsec.m -> pdf(-ray.direction, wo, N);
                Vector3f f = ray_intsec.m -> eval(-ray.direction, wo, N, depth == 0, Importance);

                auto outray_origin = ray_intsec.coords + (dotProduct(wo, N) > 0 ? EPSILON * N : -EPSILON * N);
                ray = Ray(outray_origin, wo);

                // update power_scale based on rendering equation
                power_scale = power_scale * f;
                // power_scale = power_scale * abs_cos_theta(-ray.direction, N) * f;
                power_scale = power_scale/RussianRoulette;
            } else {
                // did not pass the RR test
                break;
            }
        }
    }

    // balance the photon map, so that it could be queried
    pm.balance();
}
