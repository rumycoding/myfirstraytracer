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

Bounds3 BVHAccel::WorldBound() const{
    if (!root) return Bounds3();

    return root->bounds;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
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
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        std::vector<Object*> leftshapes, rightshapes;

        if (splitMethod == SplitMethod::NAIVE) {
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
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            leftshapes = std::vector<Object*>(beginning, middling);
            rightshapes = std::vector<Object*>(middling, ending);
        } else {
            BucketInfo buckets[bucketSize];

            for (int i = 0; i < objects.size(); i++ ) {
                int b = 0;
                Vector3f objectCenter = objects[i]->getBounds().Centroid();
                b = (objectCenter[dim] - centroidBounds.pMin[dim]) / (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]) * (float)bucketSize;
                
                if (std::isnan(b)) b = 0;
                if (b < 0) b = 0;
                if (b > bucketSize - 1) b = bucketSize - 1;

                buckets[b].count += 1;
                buckets[b].bounds = Union(buckets[b].bounds, objects[i]->getBounds());
            }

            // Calculate SAH
            float minCost = std::numeric_limits<float>::infinity();
            int minIndex = -1;
            Bounds3 forwardBound;
            int forwardCount = 0;
            int backwardCount = objects.size();

            double overallSurfaceArea = centroidBounds.SurfaceArea();

            for (int i = 0; i < bucketSize - 1; i++) {
                forwardBound = Union(forwardBound, buckets[i].bounds);
                forwardCount += buckets[i].count;
                backwardCount -= buckets[i].count;
                
                Bounds3 backwardBound;
                for (int j = i+1; j<bucketSize; j++) {
                    backwardBound = Union(backwardBound, buckets[j].bounds);
                }

                // I removed entries that are not needed, should be equivalent.
                float cost = (forwardCount * forwardBound.SurfaceArea() + backwardCount * backwardBound.SurfaceArea());

                if (cost < minCost) {
                    minCost = cost;
                    minIndex = i;
                }
            }

            leftshapes = std::vector<Object*>();
            rightshapes = std::vector<Object*>();

            for (int i = 0; i < objects.size(); i++ ) {
                int b = 0;
                Vector3f objectCenter = objects[i]->getBounds().Centroid();
                switch (dim)
                {
                    case 0:
                        b =  (objectCenter.x - centroidBounds.pMin.x) / (centroidBounds.pMax.x - centroidBounds.pMin.x) * (float)bucketSize;
                        break;
                    case 1:
                        b =  (objectCenter.y - centroidBounds.pMin.y) / (centroidBounds.pMax.y - centroidBounds.pMin.y) *  (float)bucketSize;
                        break;
                    case 2:
                        b =  (objectCenter.z - centroidBounds.pMin.z) / (centroidBounds.pMax.z - centroidBounds.pMin.z) *  (float)bucketSize;
                        break;
                }

                if (std::isnan(b)) b = 0;
                if (b < 0) b = 0;
                if (b > bucketSize - 1) b = bucketSize - 1;

                if (b <= minIndex) {
                    leftshapes.push_back(objects[i]);
                } else {
                    rightshapes.push_back(objects[i]);
                }
            }

            // no need to split, but we have to split....
            if (leftshapes.size() == 0 || rightshapes.size() == 0) {
                // keep build

                // divide left right to equally distributed half
                int median = objects.size()/2;

                std::vector<Object*> leftHalf;
                leftHalf.insert(leftHalf.begin(), objects.begin(), objects.begin() + median);
                std::vector<Object*> rightHalf;
                rightHalf.insert(rightHalf.begin(), objects.begin() + median, objects.end());

                node->left = recursiveBuild(leftHalf);
                node->right = recursiveBuild(rightHalf);

                node->bounds = Union(node->left->bounds, node->right->bounds);

                return node;
            }

        }
    
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
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
    if (!node) return Intersection();

    // Traverse the BVH to find intersection
    bool intersect = node -> bounds.IntersectP(ray);

    if (!intersect) return Intersection();

    // does intersect
    if (node -> object) {
        return node -> object -> getIntersection(ray);
    }

    auto leftIntersection = getIntersection(node->left, ray);
    auto rightIntersection = getIntersection(node->right, ray);

    return leftIntersection.distance < rightIntersection.distance ? leftIntersection : rightIntersection;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}