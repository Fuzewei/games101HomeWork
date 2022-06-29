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
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
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
    Vector3f hitColor = this->backgroundColor;
    Intersection intersection = Scene::intersect(ray);
    if(intersection.happened){
        if (intersection.m->hasEmission())
        {
           return intersection.m->getEmission();
        }
        
        hitColor = shade(intersection, -ray.direction);
    } 
    return hitColor;
}


Vector3f Scene::shade(const Intersection &p, const Vector3f &wo)const
{
    Vector3f l_light_dir, l_in_dir;
    Intersection lightPoint; float pdf_light;
    Scene::sampleLight(lightPoint, pdf_light);

    Vector3f lightDir = lightPoint.coords - p.coords;
    float dis = dotProduct(lightDir, lightDir);
    Ray toLight = Ray(p.coords, normalize(lightDir));
    Intersection intersection = Scene::intersect(toLight);

    if (intersection.happened && (intersection.coords - lightPoint.coords).norm() < 1e-2)
    {
        Vector3f f_r = p.m->eval(-wo, toLight.direction, p.normal);
        l_light_dir = lightPoint.emit * f_r * dotProduct(-toLight.direction, lightPoint.normal)*
                     dotProduct(toLight.direction, p.normal) / dis /pdf_light;
        
    }


    if (get_random_float() < RussianRoulette)
    {
        Vector3f wi = p.m->sample(-wo, p.normal).normalized();
        Ray toObj = Ray(p.coords, wi);
        Intersection intersection = intersect(toObj);
        if (intersection.happened && !intersection.m->hasEmission())
        {
            float pdf = p.m->pdf(-wo, wi, p.normal);
            Vector3f f_r = p.m->eval(-wo, wi, p.normal);
            l_in_dir = shade(intersection, -wi) * f_r * 
                    dotProduct(wi ,p.normal)/pdf/RussianRoulette;
        }
        
    }

    return l_light_dir + l_in_dir;
}