#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

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
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
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
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (splitMethod)
        {
        case SplitMethod::NAIVE:
        {
            switch (dim)
            {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().x <
                                   f2->getBounds().Centroid().x; });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().y <
                                   f2->getBounds().Centroid().y; });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().z <
                                   f2->getBounds().Centroid().z; });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object *>(beginning, middling);
            auto rightshapes = std::vector<Object *>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        break;

        case SplitMethod::SAH:
        {
            float SN = centroidBounds.SurfaceArea();
            switch (dim)
            {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().x <
                                   f2->getBounds().Centroid().x; });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().y <
                                   f2->getBounds().Centroid().y; });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                          { return f1->getBounds().Centroid().z <
                                   f2->getBounds().Centroid().z; });
                break;
            }

            int bin_count = 10;
            int best_split_index = 0;
            float minCost = std::numeric_limits<float>::infinity();

            for (int i = 0; i < bin_count; i++)
            {
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * i / bin_count);
                auto ending = objects.end();
                auto leftshapes = std::vector<Object *>(beginning, middling);
                auto rightshapes = std::vector<Object *>(middling, ending);

                Bounds3 leftBounds, rightBounds;
                for (int k = 0; k < leftshapes.size(); ++k)
                {
                    leftBounds = Union(leftBounds, leftshapes[k]->getBounds().Centroid());
                }

                for (int k = 0; k < rightshapes.size(); ++k)
                {
                    rightBounds = Union(rightBounds, rightshapes[k]->getBounds().Centroid());
                }

                float pLeft = leftBounds.SurfaceArea() / centroidBounds.SurfaceArea();
                float pRight = rightBounds.SurfaceArea() / centroidBounds.SurfaceArea();
                float cost = 0.125 + pLeft * leftshapes.size() + pRight * rightshapes.size();
                if (cost < minCost)
                {
                    minCost = cost;
                    best_split_index = i;
                }
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * best_split_index / bin_count);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object *>(beginning, middling);
            auto rightshapes = std::vector<Object *>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        break;
        }
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
    // TODO Traverse the BVH to find intersection

    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = static_cast<int>(ray.direction.x < 0);
    dirIsNeg[1] = static_cast<int>(ray.direction.y < 0);
    dirIsNeg[2] = static_cast<int>(ray.direction.z < 0);
    Intersection inter;
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return inter;
    }

    if (node->left == nullptr && node->right == nullptr)
    {
        inter = node->object->getIntersection(ray);
        return inter;
    }

    Intersection inter_left = getIntersection(node->left, ray);
    Intersection inter_right = getIntersection(node->right, ray);
    return inter_left.distance < inter_right.distance ? inter_left : inter_right;
}