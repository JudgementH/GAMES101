//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
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
    float eps = 1e-4;
    // // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);

    if (!inter.happened)
    {
        return Vector3f(0.0f, 0.0f, 0.0f);
    }
    if (inter.m->hasEmission())
    {
        return inter.m->getEmission();
    }

    // direction light
    Vector3f l_dir(0.0f, 0.0f, 0.0f);
    Intersection light_point;
    float light_point_prob;
    sampleLight(light_point, light_point_prob);

    Ray test_ray(inter.coords, normalize(light_point.coords - inter.coords));
    Intersection test_inter = intersect(test_ray);
    if (test_inter.distance - light_point.distance < eps)
    {
        l_dir += light_point.emit *
                 dotProduct(light_point.normal, -test_ray.direction) /
                 std::pow((light_point.coords - inter.coords).norm(), 2) *
                 dotProduct(inter.normal, test_ray.direction) *
                 inter.m->eval(test_ray.direction, -ray.direction, inter.normal) /
                 light_point_prob;
    }

    // indirection light
    Vector3f l_indir(0.0f, 0.0f, 0.0f);
    float random = get_random_float();
    if (random < RussianRoulette)
    {
        Vector3f w_in = inter.m->sample(-ray.direction, inter.normal).normalized();
        Ray ray_in(inter.coords, w_in);
        Intersection inter_indir = intersect(ray_in);
        if (inter_indir.happened && (!inter_indir.m->hasEmission()))
        {
            l_indir += castRay(ray_in, depth + 1) *
                       dotProduct(inter.normal, ray_in.direction) *
                       inter.m->eval(w_in, -ray.direction, inter.normal) /
                       inter.m->pdf(w_in, -ray.direction, inter.normal) / RussianRoulette;
        }
    }
    return l_dir + l_indir;
}