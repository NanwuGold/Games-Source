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
    /// 只计算 直接光照 -- 假设不存在遮挡
    auto raySect = intersect(ray);  ///< 获取光线和场景的交点

    Vector3f L_dir = {0.0};
    /// 没有打到场景的任何物体 -- 包括光源
    if(!raySect.happened)
    {
        return L_dir;
    }

    /// 打到光源
    if(raySect.m->hasEmission())
    {
        if(depth == 0)  /// 第一次
        {
            return raySect.m->getEmission();
        }
        else
        {
            return L_dir;
        }
    }

    auto p = raySect.coords;  /// 交点
    auto N = raySect.normal.normalized();  /// 交点的法线
    auto w_o = ray.direction; w_o = w_o.normalized();

    /// 获取采样光源的PDF
    /// 获取朝向光源的光线
    auto XX = Intersection{};
    float pdf_light = 0.0;
    sampleLight(XX, pdf_light);

    auto NN = XX.normal.normalized();

    /// 着色点到光源的方向
    auto w_s =  p - XX.coords;

    auto dis_2 = dotProduct(w_s, w_s);

    w_s = w_s.normalized();

    auto emit = XX.emit; /// 光源的强度
    auto f_brdf = raySect.m->eval(w_o, -w_s, N);
    auto cos_theta =  dotProduct(-w_s, N);
    auto cos_theta_light =  dotProduct(w_s, NN);
    auto outres = emit * f_brdf * cos_theta * cos_theta_light / dis_2 / pdf_light;

    return outres;
}