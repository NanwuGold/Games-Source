#include "BVH.hpp"
#include "Vector.hpp"
#include <algorithm>
#include <cassert>
#include <unordered_map>

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

    printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveSHABuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    if (objects.size() == 1) ///< 只有一个元素的情况下 直接构建节点
    {
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->bounds = objects[0]->getBounds();

        return node;
    } else if (objects.size() == 2) ///< 两个节点 直接分开存到两个叶子节点中
    {
        node->left = recursiveSHABuild({objects[0]});
        node->right = recursiveSHABuild({objects[1]});
        node->bounds = Union(objects[0]->getBounds(), objects[1]->getBounds());
        return node;
    } else
    {
        // 计算整个场景的包围盒
        Bounds3 bounds = objects[0]->getBounds();
        for (int i = 1; i < objects.size(); ++i)
        {
            bounds = Union(bounds, objects[i]->getBounds());
        }

        auto indexset = std::vector<int>{0, 1, 2}; /// x , y, z

        /// 记录每个轴的分割平面的开销
        std::vector<std::unordered_map<int, float>> axisCasts(3);   ///< 考虑直接使用 std::vector
        const int objectsNums = objects.size();

        for (const auto index : indexset)
        {
            auto &castMap = axisCasts[index];
            std::vector<Object *> copyObjects(objects);

            /// 根据当前面计算候选分割面 - 按照质心坐标排序
            std::sort(copyObjects.begin(), copyObjects.end(), [index](auto f1, auto f2) {
                return (f1->getBounds().Centroid())[index] < (f2->getBounds().Centroid())[index];
            });

            /// 计算前缀包围盒
            std::vector<Bounds3> leftBounds(objectsNums);
            leftBounds[0] = copyObjects[0]->getBounds();
            for (auto index = 1; index < objectsNums; index++)
            {
                leftBounds[index] = Union(leftBounds[index - 1], copyObjects[index]->getBounds());
            }

            /// 计算后缀包围盒
            std::vector<Bounds3> rightBounds(objectsNums);
            leftBounds[objectsNums - 1] = copyObjects[objectsNums - 1]->getBounds();
            for (auto index = objectsNums - 2; index >= 0; index--)
            {
                rightBounds[index] = Union(rightBounds[index + 1], copyObjects[index]->getBounds());
            }

            auto SareaA = leftBounds[objectsNums - 1].SurfaceArea(); ///< 所有物体的包围盒的表面积

            /// 循环遍历分割
            for (int left = 0; left < objectsNums - 1; left++)
            {
                const auto Nleft = left + 1;                 ///< 左侧树的节点数目  -- 前N个分配给左子树(N  < objectsNums,总数)
                const auto Nright = objectsNums - Nleft;     ///< 右侧树的节点数目 剩余部分分配给右子树

                /// 计算左右子树的表面积
                auto SareaLeft = leftBounds[left].SurfaceArea();
                auto SareaRight = rightBounds[left + 1].SurfaceArea();

                auto cast = 1.0f + (SareaLeft * Nleft + SareaRight * Nright) / SareaA;

                /// 记录数目对应的开销
                castMap.insert({Nleft, cast});
            }
        }

        /// 获取最小开销
        [[maybe_unused]] auto XminumCast = std::min_element(
            axisCasts[0].begin(), axisCasts[0].end(),
            [](const std::pair<int, float> &p1, const std::pair<int, float> &p2) { return p1.second < p2.second; });

        [[maybe_unused]] auto YminumCast = std::min_element(
            axisCasts[1].begin(), axisCasts[1].end(),
            [](const std::pair<int, float> &p1, const std::pair<int, float> &p2) { return p1.second < p2.second; });

        [[maybe_unused]] auto ZminumCast = std::min_element(
            axisCasts[2].begin(), axisCasts[2].end(),
            [](const std::pair<int, float> &p1, const std::pair<int, float> &p2) { return p1.second < p2.second; });

        int axisIndex = 0;
        std::pair<int, float> res;
        res = *XminumCast;
        if (XminumCast->second < YminumCast->second && XminumCast->second < ZminumCast->second)
        {
            axisIndex = 0;
            res = *XminumCast;
        } else if (YminumCast->second < ZminumCast->second)
        {
            axisIndex = 1;
            res = *YminumCast;
        } else
        {
            axisIndex = 2;
            res = *ZminumCast;
        }

        std::sort(objects.begin(), objects.end(), [index = axisIndex](auto f1, auto f2) {
            return (f1->getBounds().Centroid())[index] < (f2->getBounds().Centroid())[index];
        });

        auto beginning = objects.begin();
        auto middling = objects.begin() + res.first;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling); ///< 左闭右开原则
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveSHABuild(leftshapes);
        node->right = recursiveSHABuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
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
    } else if (objects.size() == 2) ///< 存在多个物体 则继续创建节点
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else
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
            std::sort(objects.begin(), objects.end(),
                      [](auto f1, auto f2) { return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x; });
            break;
        case 1: ///< 切 Y 方向
            std::sort(objects.begin(), objects.end(),
                      [](auto f1, auto f2) { return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y; });
            break;
        case 2: ///< 切 Z 方向
            std::sort(objects.begin(), objects.end(),
                      [](auto f1, auto f2) { return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z; });
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
        if (auto obj = node->object; !(node->left) && !(node->right) && obj) /// 此时是一个叶子节点 -- 存储数据 计算求交
        {
            return obj->getIntersection(ray);
        }

        auto hit_1 = getIntersection(node->left, ray);
        auto hit_2 = getIntersection(node->right, ray);

        if (hit_1.distance < hit_2.distance)
        {
            return hit_1;
        } else
        {
            return hit_2;
        }
    }

    return Intersection();
}