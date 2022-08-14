#include <algorithm>
#include <cassert>
#include "BVH.hpp"


BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;
    if (splitMethod == SplitMethod::NAIVE) {
        root = recursiveBuild(primitives);
    } else if (splitMethod == SplitMethod::SAH) {
        root = recursiveBuildWithSAH(primitives);
    } else {
        throw std::runtime_error("Split method error!");
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

//BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
//{
//    BVHBuildNode* node = new BVHBuildNode();
//
//    // Compute bounds of all primitives in BVH node
//    Bounds3 bounds;
//    for (int i = 0; i < objects.size(); ++i)
//        bounds = Union(bounds, objects[i]->getBounds());
//    if (objects.size() == 1) {
//        // Create leaf _BVHBuildNode_
//        node->bounds = objects[0]->getBounds();
//        node->object = objects[0];
//        node->left = nullptr;
//        node->right = nullptr;
//        return node;
//    }
//    else if (objects.size() == 2) {
//        node->left = recursiveBuild(std::vector{objects[0]});
//        node->right = recursiveBuild(std::vector{objects[1]});
//
//        node->bounds = Union(node->left->bounds, node->right->bounds);
//        return node;
//    }
//    else {
//        Bounds3 centroidBounds;
//        for (int i = 0; i < objects.size(); ++i)
//            centroidBounds =
//                Union(centroidBounds, objects[i]->getBounds().Centroid());
//        int dim = centroidBounds.maxExtent();
//        switch (dim) {
//        case 0:
//            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                return f1->getBounds().Centroid().x <
//                       f2->getBounds().Centroid().x;
//            });
//            break;
//        case 1:
//            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                return f1->getBounds().Centroid().y <
//                       f2->getBounds().Centroid().y;
//            });
//            break;
//        case 2:
//            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                return f1->getBounds().Centroid().z <
//                       f2->getBounds().Centroid().z;
//            });
//            break;
//        }
//
//        auto beginning = objects.begin();
//        auto middling = objects.begin() + (objects.size() / 2);
//        auto ending = objects.end();
//
//        auto leftshapes = std::vector<Object*>(beginning, middling);
//        auto rightshapes = std::vector<Object*>(middling, ending);
//
//        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
//
//        node->left = recursiveBuild(leftshapes);
//        node->right = recursiveBuild(rightshapes);
//
//        node->bounds = Union(node->left->bounds, node->right->bounds);
//    }
//
//    return node;
//}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    // 通过树形结构，划分物体以此划分包围盒
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    //计算根节点的所有bounds
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
        // 交换选择纬度错切分，划分子节点
        int dim = centroidBounds.maxExtent();
        switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                           f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                           f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                           f2->getBounds().Centroid().z;
                });
                break;
        }

        // 递归分离节点
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        bool SAH = true;

        if(SAH){
            // 递归分离节点
            auto size = objects.size();
            int proper_cut = 0;
            double mintime = 0x3f3f3f;
            for(int index=0; index<size; index++){
                middling = objects.begin() + index;

                Bounds3 leftBounds,rightBounds;
                //     time = S_1面积 /S_0面积 *S_1空间物体数 * t_obj
                //              + S_2面积 /S_0面积 *S_2空间物体数 * t_obj
                for (int i = 0; i != index; ++i)
                    leftBounds =
                            Union(leftBounds, objects[i]->getBounds().Centroid());
                for (int i = index; i < objects.size(); ++i)
                    rightBounds =
                            Union(rightBounds, objects[i]->getBounds().Centroid());

                auto leftS = leftBounds.SurfaceArea();
                auto rightS = rightBounds.SurfaceArea();
                auto S = leftS + rightS;
                auto time = leftS / S * index + rightS / S * (objects.size() - index);
                if(time<mintime){
                    mintime = time;
                    proper_cut = index;
                }
            }
//            std::cout << proper_cut << '\n';
            middling = objects.begin() + proper_cut;
        }

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}



float cost(const std::vector<Object *>& objects, size_t split_index) {
    Bounds3 leftBounds,rightBounds;
    //     time = S_1面积 /S_0面积 *S_1空间物体数 * t_obj
    //              + S_2面积 /S_0面积 *S_2空间物体数 * t_obj
    for (int i = 0; i != split_index; ++i)
        leftBounds =
                Union(leftBounds, objects[i]->getBounds().Centroid());
    for (int i = split_index; i < objects.size(); ++i)
        rightBounds =
                Union(rightBounds, objects[i]->getBounds().Centroid());

    auto leftS = leftBounds.SurfaceArea();
    auto rightS = rightBounds.SurfaceArea();
    auto S = leftS + rightS;
    auto time = leftS / S * split_index + rightS / S * (objects.size() - split_index);
    return time;
}


//std::pair<std::vector<Object*>, size_t > minCost(std::vector<Object *> objects) {
//    float total_cost(std::numeric_limits<float>::max()), current_cost(0);
//    size_t split_index(0);
//    int split_dim(0);
//    // 对x轴排序
//    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//        return f1->getBounds().Centroid().x <
//               f2->getBounds().Centroid().x;
//    });
//    for (size_t index = 0; index != objects.size() - 1; ++index) {
//        current_cost = cost(objects[0]->getBounds().Centroid(), objects[index]->getBounds().Centroid(),
//                            objects[index + 1]->getBounds().Centroid(), objects[objects.size() - 1]->getBounds().Centroid(),
//                            index + 1, objects.size() - index - 1);
//        if (current_cost < total_cost) {
//            total_cost = current_cost;
//            split_dim = 0;
//            split_index = index;
//        }
//    }
//    // 对y轴排序
//    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//        return f1->getBounds().Centroid().y <
//               f2->getBounds().Centroid().y;
//    });
//    for (size_t index = 0; index != objects.size() - 1; ++index) {
//        current_cost = cost(objects[0]->getBounds().Centroid(), objects[index]->getBounds().Centroid(),
//                            objects[index + 1]->getBounds().Centroid(), objects[objects.size() - 1]->getBounds().Centroid(),
//                            index + 1, objects.size() - index - 1);
//        if (current_cost < total_cost) {
//            total_cost = current_cost;
//            split_dim = 1;
//            split_index = index;
//        }
//    }
//    // 对z轴排序
//    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//        return f1->getBounds().Centroid().z <
//               f2->getBounds().Centroid().z;
//    });
//    for (size_t index = 0; index != objects.size() - 1; ++index) {
//        current_cost = cost(objects[0]->getBounds().Centroid(), objects[index]->getBounds().Centroid(),
//                            objects[index + 1]->getBounds().Centroid(), objects[objects.size() - 1]->getBounds().Centroid(),
//                            index + 1, objects.size() - index - 1);
//        if (current_cost < total_cost) {
//            total_cost = current_cost;
//            split_dim = 2;
//            split_index = index;
//        }
//    }
//    if (split_dim == 2) {
//        return std::make_pair(objects, split_index);
//    }
//    switch (split_index) {
//        case 0:
//            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                return f1->getBounds().Centroid().x <
//                       f2->getBounds().Centroid().x;
//            });
//            break;
//        case 1:
//            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
//                return f1->getBounds().Centroid().y <
//                       f2->getBounds().Centroid().y;
//            });
//            break;
//    }
//    return std::make_pair(objects, split_index);
//}


std::pair<std::vector<Object*>, size_t > minCost(std::vector<Object *> objects) {
    float total_cost(std::numeric_limits<float>::max()), current_cost(0);
    size_t split_index(0);
    int split_dim(0);
    for (size_t index = 0; index != objects.size() - 1; ++index) {
        current_cost = cost(objects, index);
        if (current_cost < total_cost) {
            total_cost = current_cost;
            split_index = index;
        }
    }
    return std::make_pair(objects, split_index);
}


BVHBuildNode *BVHAccel::recursiveBuildWithSAH(std::vector<Object *> objects) {

    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuildWithSAH(std::vector{objects[0]});
        node->right = recursiveBuildWithSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    } else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
        // 交换选择纬度错切分，划分子节点
        int dim = centroidBounds.maxExtent();
        switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                           f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                           f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                           f2->getBounds().Centroid().z;
                });
                break;
        }
        auto result = minCost(objects);
//        std::cout << result.second << '\n';
        auto leftshapes = std::vector<Object*>(result.first.begin(), result.first.begin() + result.second + 1);
        auto rightshapes = std::vector<Object*>(result.first.begin() + result.second + 1, result.first.end());

        node->left = recursiveBuildWithSAH(leftshapes);
        node->right = recursiveBuildWithSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    if (!(node->bounds.IntersectP(ray, ray.direction_inv, {0, 0, 0}))) {
        return Intersection();
    }
    if (!(node->left || node->right)) {
        return node->object->getIntersection(ray);
    }
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);
    return hit1.distance < hit2.distance ? hit1 : hit2;
}



//Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
//{
//    Intersection inter;
//    //invdir = 1 / D; bounds3.hpp中会用到。
//    Vector3f invdir(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
//    //判断射线的方向正负，如果负，为1；bounds3.hpp中会用到。
//    std::array<int, 3> dirIsNeg;
//    dirIsNeg[0] = ray.direction.x < 0;
//    dirIsNeg[1] = ray.direction.y < 0;
//    dirIsNeg[2] = ray.direction.z < 0;
//
//    //没有交点
//    if(!node -> bounds.IntersectP(ray, invdir, dirIsNeg))
//        return inter;
//    //有交点，且该点为叶子节点，去和三角形求交
//    if(node -> left == nullptr && node -> right == nullptr)
//        return node -> object -> getIntersection(ray);
//    //该点为中间节点，继续判断，并返回最近的包围盒交点
//    Intersection hit1 = getIntersection(node -> left,  ray);
//    Intersection hit2 = getIntersection(node -> right, ray);
//    return hit1.distance < hit2.distance ? hit1 : hit2;
//}
