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

float FresnelTerm(const Vector3f wo, const Vector3f N, float n1, float n2) {
    float R0(std::pow((n1 - n2) / (n1 + n2), 2));
    float cos0(dotProduct(-wo, N));
    return R0 + (1 - R0) * std::pow(1 - cos0, 5);
}

Vector3f shade(const Intersection& inter, const Vector3f& wo, const Scene& scene) {
    float pdf_light = 1.0, RussianRoulette = 0.8;
    Vector3f L_dir(0.0, 0.0, 0.0), L_indir(0.0, 0.0, 0.0);
    Intersection light_inter;
    Vector3f p(inter.coords), N(inter.normal);
    scene.sampleLight(light_inter, pdf_light);
    Vector3f light_pos(light_inter.coords), ws((inter.coords - light_pos).normalized()), NN(light_inter.normal),
    emit(light_inter.emit), wi_light(ws * -1.0);
    Ray p_to_light(inter.coords, wi_light);
    Intersection ll(scene.intersect(p_to_light));
    float dd((light_pos - p).norm());
    // 判断是否无遮挡
    if (std::abs(ll.distance - dd) < 0.005) {
        #ifndef MicrofacetBRDF
            const Vector3f f_r(inter.m->eval(wo, wi_light, N));
        #else
            // TODO Microfacet BRDF
            float fresnel_term(FresnelTerm(wo, inter.normal, 1.0,  inter.m->ior));
        #endif
        L_dir = emit * f_r * dotProduct(wi_light, N) * dotProduct(ws, NN)
                / std::pow((light_pos - p).norm(), 2) / pdf_light;
    }

    if (get_random_float() < RussianRoulette) {
        Vector3f wi = inter.m->sample(wo, inter.normal).normalized();
        Ray p_to_q(p, wi);
        Intersection inter_p_to_q(scene.intersect(p_to_q));
        if (inter_p_to_q.happened && !(inter_p_to_q.m->hasEmission())) {
            #ifndef MicrofacetBRDF
                const Vector3f f_r(inter.m->eval(wo, wi, N));
            #else
                // TODO Microfacet BRDF
            float fresnel_term(FresnelTerm(wo, inter.normal, 1.0,  inter.m->ior));
            
            #endif
            L_indir = scene.castRay(p_to_q, 0) * f_r *
                      dotProduct(wi, inter.normal) / inter.m->pdf(wo, wi, inter.normal) /
                      RussianRoulette;
        }
    }
    return L_dir + L_indir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection inter = intersect(ray);
    if (inter.happened) {
        if (inter.m->hasEmission()) {
            return inter.m->getEmission();
        }
        return shade(inter, ray.direction, *this);
    }
    return Vector3f(0.0, 0.0, 0.0);
}