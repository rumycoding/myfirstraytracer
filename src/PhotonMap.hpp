#pragma once
#include "Vector.hpp"
#include "queue"
#include "Bounds3.hpp"
#include "Material.hpp"

enum PhotonMapType {GLOBAL, CAUSTIC};

struct Photon
{
    Vector3f pos;
    Vector3f dir;   // wi, pointing inwards the plane
    Vector3f power;
    // a photon will be used to split other photons
    // will be used when buidling kd-tree
    short plane=3;    // split plane, 0=x, 1=y, 2=z, 3=uninitialized
};

struct NearestPhotonEntry {
    const Photon* photon;
    float dist2;

    // make a max heap
    bool operator <(const NearestPhotonEntry& rhs) const {
        return dist2 < rhs.dist2;
    }
};

class NearestPhotons {
public:
    NearestPhotons(const Vector3f &p, const int N, const float a): pos(p), maxNumPhotons(N), acceptedMaxDist2(a) {
    }

    std::priority_queue<NearestPhotonEntry> maxheap;
    Vector3f pos;

    // if photon distance is larger than accepted max, it will be rejected
    // will init it with a fairly large number
    float acceptedMaxDist2;

    // max number of photons inside the heap
    int maxNumPhotons;

    void pushPhotonToHeap(const Photon* p) {
        float dist2 = (p->pos - pos).dist2();

        if (dist2 < acceptedMaxDist2) {
            if (maxheap.size() < maxNumPhotons) {
                maxheap.push({p, dist2});
            } else if (dist2 < getCurrentMaxDist2()) {
                // max heap is full, need to pop
                maxheap.pop();
                maxheap.push({p, dist2});
            }

            // update the prune number to guide searching
            if (maxheap.size() == maxNumPhotons) {
                acceptedMaxDist2 = getCurrentMaxDist2();
            }
        }
    }

    float getCurrentMaxDist2() {
        if (maxheap.size() == 0) return std::numeric_limits<float>::infinity();

        return maxheap.top().dist2;
    }

    size_t getSize() {
        return maxheap.size();
    }
};

class PhotonMap {
public:
    PhotonMap(int m, int n=100, PhotonMapType pm_type=PhotonMapType::GLOBAL);
    ~PhotonMap();

    // store a photon into the photon map
    void store(Vector3f pos, Vector3f dir, Vector3f power);

    // balance the kd-tree
    void balance();

    // estimate the irradiance at certain position
    // wo is the eye direction, pointing outwards the point
    Vector3f radianceEstimate(const Vector3f& pos, const Vector3f& normal, const Vector3f& wo, Material* m) const;

    PhotonMapType type;

    void scale(float s) {
        for (auto p: photons) {
            if (!(p == nullptr)) {
                p->power = s * p->power;
            }
        }
    }

    int size() {
        return photons.size();
    }
private:
    std::vector<Photon*> photons;

    // will find N nearest neighbors
    const int N = 100;
    const float maxDist = 10.0f;

    // balance a segment in orignalVector, put it into the result vector
    void balanceSegment(std::vector<Photon*> &resultVector, std::vector<Photon*> &originalVector, const int index, const int start, const int end);

    // after median split, in the origianal vector [start, end), 
    // [start, median) will be smaller than median, 
    // (median, end) will be larger than median.
    void medianSplit(std::vector<Photon*> &originalVector, int start, int end, int median);

    // locate N nearest photons
    void locatePhotons(NearestPhotons &np, const int index) const;

    // get guassian weight
    // not used...
    float gaussianFilter(float dp2, float maxDistance2, float a=0.918, float b=1.953) const {
        return a * (1.0f-(1.0f-exp(-b * dp2/ (2.0f * maxDistance2)))/(1.0f-exp(-b)));
    }

};

#pragma region randomizedSelect
int randomizedPartition(std::vector<Photon*> &originalVector, int start, int end, int dim);

void randomizedSelect(std::vector<Photon*> &originalVector, int start, int end, int i, int dim);
#pragma endregion


#pragma region treeHelper
// tree with height 2 has 3 nodes, if it is complete
inline int completeTreeSizeWithHeight(int h){
    return (1 << h) - 1;
}

// tree level 0 has 1 node, tree level 1 has 2 node
inline int nodeSizeInLevel(int l) {
    return (1 << l);
}

inline int getLeftTreeNodeIndex(int index) {
    return 2 * index + 1;
}

inline int getRightTreeNodeIndex(int index) {
    return 2 * index + 2;
}
#pragma endregion