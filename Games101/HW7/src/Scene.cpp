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

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects, float &tNear, uint32_t &index,
                  Object **hitObject)
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
    /// 只计算 直接光照 -- 假设不存在遮挡
    auto shadingPointInter = intersect(ray); ///< 获取光线和场景的交点

    Vector3f L_dir = {0.0}, L_indir{0.0f};

    /// 没有打到场景的任何物体 -- 包括光源
    if (!shadingPointInter.happened)
    {
        return {0.0}; /// 直接着色为背景色
    }

    if (shadingPointInter.obj->hasEmit() && depth == 0)
    {
        return shadingPointInter.m->getEmission();
    }

    auto p = shadingPointInter.coords; /// 交点
    auto N = shadingPointInter.normal; /// 交点的法线
    auto w_o = normalize(-ray.direction);

    /// 计算直接光照
    {
        /// 获取采样光源的PDF
        /// 获取朝向光源的光线
        Intersection LightInter;
        float pdf_light;
        sampleLight(LightInter, pdf_light);

        auto x = LightInter.coords;
        auto dir = x - p;
        auto dis = dir.norm();
        auto dis_2 = dis * dis;

        auto w_s = dir.normalized(); /// 着色点到光源的方向
        auto NN = LightInter.normal;
        auto emit = LightInter.emit;

        auto ray2Light = Ray(p, w_s);

        /// 计算直接光照的光线是否被遮挡
        auto lightBlackInter = intersect(ray2Light);
        if (lightBlackInter.happened && std::abs((lightBlackInter.coords - x).norm()) < 0.0001)
        {
            auto f_brdf = shadingPointInter.m->eval(w_o, w_s, N);   ///< 使用diffuse材质 和出射角度无关
            auto cos_theta = std::max(0.0f, dotProduct(w_s, N));
            auto cos_theta_light = std::max(0.0f, dotProduct(-w_s, NN));
            L_dir = emit * f_brdf * cos_theta * cos_theta_light / (dis_2 * pdf_light);
        }
    }

    /// 计算间接光照
    {
        auto random_pdf = get_random_float();
        if (random_pdf > RussianRoulette)
        {
            /// 俄罗斯轮盘赌 死亡
            L_indir = {0.0};
        }
        else
        {
            auto w_i = shadingPointInter.m->sample(w_o, N).normalized();  ///< 得到采样方向
            auto in_ray = Ray(p, w_i);

            auto pdf = shadingPointInter.m->pdf(w_o, w_i, N);
            L_indir = castRay(in_ray, depth + 1) * shadingPointInter.m->eval(w_o, w_i, N) * dotProduct(w_i, N) / (pdf * RussianRoulette);
        }
    }

    /// TODO: implement Microfacet BRDF

    return L_dir + L_indir;
}