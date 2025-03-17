#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
    {
        return;
    }

    switch (splitMethod)
    {
    case SplitMethod::NAIVE:
        root = recursiveBuild(primitives);
        break;
    case SplitMethod::SAH:
        root = recursiveSHABuild(primitives);
        break;
    default:
        break;
    }

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // 计算整个场景的包围盒
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
    {
        bounds = Union(bounds, objects[i]->getBounds());
    }

    /// 只有一个物体 则直接创建为叶子节点 记录包围盒&物体
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) ///< 存在多个物体 则继续创建节点
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

        int dim = centroidBounds.maxExtent();
        /// 计算切分的轴向，根据质心坐标对应轴向的值从小到大进行排序
        switch (dim)
        {
        case 0: ///< 切 X 方向
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x; });
            break;
        case 1: ///< 切 Y 方向
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y; });
            break;
        case 2: ///< 切 Z 方向
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z; });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling); ///< 左闭右开原则
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode *BVHAccel::recursiveSHABuild(std::vector<Object *> objects)
{
    return nullptr;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    if (!node) ///< 节点为空的时候
    {
        return {};
    }

    const auto &[x, y, z] = ray.direction;
    if (node->bounds.IntersectP(ray, ray.direction_inv, {int(x > 0), int(y > 0), int(z > 0)}))
    {
        if (auto obj = node->object;
            !(node->left) && !(node->right) && obj) /// 此时是一个叶子节点 -- 存储数据 计算求交
        {
            return obj->getIntersection(ray);
        }

        auto hit_1 = getIntersection(node->left, ray);
        auto hit_2 = getIntersection(node->right, ray);

        if (hit_1.distance < hit_2.distance)
        {
            return hit_1;
        }
        else
        {
            return hit_2;
        }
    }

    return Intersection();
}