//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel
{

public:
    // BVHAccel Public Types
    enum class SplitMethod
    {
        NAIVE,
        SAH
    };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode *root;

    // BVHAccel Private Methods
    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);
    BVHBuildNode *recursiveSHABuild(std::vector<Object *> objects);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object *> primitives;
};

struct BVHBuildNode
{
    Bounds3 bounds;      ///< 当前节点的包围盒
    BVHBuildNode *left;  ///< 左节点点
    BVHBuildNode *right; ///< 右结点
    Object *object;      ///< 存储的对象 -- 只有左右节点为空的时候这个才会记录，否则为空

public:
    int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
    // BVHBuildNode Public Methods
    BVHBuildNode()
    {
        bounds = Bounds3();
        left = nullptr;
        right = nullptr;
        object = nullptr;
    }
};

#endif // RAYTRACING_BVH_H
