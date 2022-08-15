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



Vector3f shade(const Intersection& inter, const Vector3f& wo, const Scene& scene) {
    float pdf_light = 1.0, RussianRoulette = 0.8;
    Vector3f L_dir, L_indir;
    Intersection light_inter;
    Vector3f p(inter.coords), N(inter.normal);
    scene.sampleLight(light_inter, pdf_light);
    if (light_inter.happened) {
        Vector3f light_pos(light_inter.coords), ws((inter.coords - light_pos).normalized()), NN(light_inter.normal),
                 emit(light_inter.emit), wi_light(ws * -1.0);
        Ray p_to_light(inter.coords, wi_light);
        // 判断是否无遮挡
        if (scene.intersect(p_to_light).distance - (light_pos - p).norm() > -0.05) {

            L_dir = emit * light_inter.m->eval(wo, wi_light, N) * dotProduct(wi_light, N) * dotProduct(ws, NN)
                    / std::pow((light_pos - p).norm(), 2) / pdf_light;
        }
    }


    if (get_random_float() < RussianRoulette) {
        Vector3f wi = inter.m->sample(wo, inter.normal).normalized();
        Ray p_to_q(p, wi);
        Intersection inter_p_to_q(scene.intersect(p_to_q));
        if (inter_p_to_q.happened && !(inter_p_to_q.m->hasEmission())) {
            L_indir = shade(inter_p_to_q, wi, scene) * inter_p_to_q.m->eval(wo, wi, N) *
                      dotProduct(-wi, inter_p_to_q.normal) / inter_p_to_q.m->pdf(wo, wi, inter_p_to_q.normal) /
                      RussianRoulette;
        } else  {
            L_indir = scene.backgroundColor;
        }
    }
    return L_dir + L_indir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    Vector3f hitColor = backgroundColor;
    Intersection inter = intersect(ray);
    if (inter.happened) {
        if (inter.m->getType() == DIFFUSE) {
            return shade(inter, ray.direction, *this);
        }
    }
    return hitColor;
}