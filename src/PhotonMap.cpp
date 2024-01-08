#include "PhotonMap.hpp"

PhotonMap::PhotonMap(int m, int n, PhotonMapType pm_type): N(n), type(pm_type) {
    photons.reserve(m);
}

PhotonMap::~PhotonMap() {
    for (auto p: photons) {
        if (p != nullptr)
            delete p;
    }
}

// store a photon into the photon map
void PhotonMap::store(Vector3f pos, Vector3f dir, Vector3f power) {
    Photon* newPhoton = new Photon{pos, dir, power};

    photons.push_back(newPhoton);
}

// balnce the kd-tree, using the median, the final representation
// will be a balance tree array representation
void PhotonMap::balance() {
    if (photons.size() > 1) {

        std::vector<Photon*> balanceResult(photons.size(), nullptr);

        balanceSegment(balanceResult, photons, 0, 0, photons.size());

        photons = std::move(balanceResult);
    }
}


void PhotonMap::balanceSegment(std::vector<Photon*> &resultVector, std::vector<Photon*> &originalVector, const int index, const int start, const int end) {

    // compute new median
    // will build the final balanced result as a 
    // complete binary tree, so will need to compute
    // median

    int median = 0;
    // tree with 8 nodes have height 4
    int treeHeight = log2(end - start) + 1;
    int treeSizeExceptLastLevel = completeTreeSizeWithHeight(treeHeight - 1);
    int lastLevelNodes = end - start - treeSizeExceptLastLevel;

    int lastLevelFullSize = nodeSizeInLevel(treeHeight - 1); 

    // if last level left half is not full
    if (lastLevelNodes < (lastLevelFullSize >> 1)) {
        median = end - start - 1 - completeTreeSizeWithHeight(treeHeight - 2);
    } else {
        // last level left half is full
        median = treeSizeExceptLastLevel;
    }

    // convert median index to vector index
    median += start;

    medianSplit(originalVector, start, end, median);

    resultVector[index] = originalVector[median];

    if (median > start) {
        int leftChildIndex = getLeftTreeNodeIndex(index);
        if ( start < median - 1) {
            balanceSegment(resultVector, originalVector, leftChildIndex, start, median);
        } else {
            // median -1 =< start < median
            // start = median - 1
            resultVector[leftChildIndex] = originalVector[start];
        }
    }

    // median + 1 is still valid in range
    if (median + 1 < end) {
        int rightChildIndex = getRightTreeNodeIndex(index);
        if (median + 1 < end - 1) {
            balanceSegment(resultVector, originalVector, rightChildIndex, median + 1, end);
        } else {
            resultVector[rightChildIndex] = originalVector[median + 1];
        }
    }
}

// actually same algorithm as the bvh class, but implementing with array, so the logic becomes harder to understand.
// I think it is a good practice for me.
// this function will modify the original vector, 
// and put the median index in `median`
// also, it will modify the Photon.plane for the median photon
void PhotonMap::medianSplit(std::vector<Photon*> &originalVector, int start, int end, int median) {
    // this is trivial
    if (end - start < 2) return;

    // find the axis to split
    Bounds3 bounds;
    for (int i = start; i < end; i++) {
        bounds = Union(bounds, originalVector[i]->pos);
    }

    int dim = bounds.maxExtent();

    randomizedSelect(originalVector, start, end, median - start, dim);

    originalVector[median]->plane = dim;
}

void PhotonMap::locatePhotons(NearestPhotons &np, const int index) const {
    const Photon* p = photons[index];

    if (p->plane < 3) {
        // p is a splitting photon

        int dim = p->plane; 
        const int leftChildIndex = getLeftTreeNodeIndex(index);
        const int rightChildIndex = getRightTreeNodeIndex(index);
        float signedDistance = np.pos[dim] - p->pos[dim];
        float squaredSignedInstance = signedDistance * signedDistance;

        if (signedDistance < 0) {
            // search left tree
            if (leftChildIndex < photons.size())
                locatePhotons(np, leftChildIndex);

            if (squaredSignedInstance < np.acceptedMaxDist2 && rightChildIndex < photons.size())
                locatePhotons(np, rightChildIndex);
        } else {
            // search right tree
            if (rightChildIndex < photons.size())
                locatePhotons(np, rightChildIndex);

            if (squaredSignedInstance < np.acceptedMaxDist2 && leftChildIndex < photons.size())
                locatePhotons(np, leftChildIndex);
        }
    }

    // push the photon into the heap
    np.pushPhotonToHeap(p);
}

Vector3f PhotonMap::radianceEstimate(const Vector3f& pos, const Vector3f& normal, const Vector3f& wo, Material* m) const {
    Vector3f result;

    NearestPhotons np(pos, N, maxDist * maxDist);

    locatePhotons(np, 0);

    // I just copied the code from Realistic Image Synthesis book
    // I guess if too fewer photon is found, it might be 
    // that the estimation is not accurate
    if (np.getSize() < 8)
        return result;

    float maxDist2 = np.getCurrentMaxDist2();

    // iterating the maxheap
    while (!np.maxheap.empty()) {
        const Photon* p = np.maxheap.top().photon;

        // p->dir should be insert into a diffuse plane
        if (dotProduct(p->dir, normal) < 0) {
            Vector3f f = m->eval(-p->dir, wo, normal);
            // should add the brdf term here
            result = result + f * p->power;
        }

        np.maxheap.pop();
    }

    result = result / (M_PI * maxDist2);

    return result;
}

#pragma region randomizedSelect
// will partition the array, and return index of the pivot
int randomizedPartition(std::vector<Photon*> &originalVector, int start, int end, int dim) {
    // will select the last element as the pivot
    int pivotIndex = start;

    Photon* pivot = originalVector[end - 1];

    // end - 1 will be the pivot
    for (int i = start; i < end - 1; i ++) {
        if (originalVector[i]->pos[dim] < pivot->pos[dim]) {
            std::swap(originalVector[i], originalVector[pivotIndex]);
            pivotIndex += 1;
        }
    }

    std::swap(originalVector[pivotIndex], originalVector[end-1]);

    return pivotIndex;
}

// implement randomized select algorithm, will put ith large elements in place
// i = 0 => minimum at start
// i = length - 1 => maximum at end 
void randomizedSelect(std::vector<Photon*> &originalVector, int start, int end, int i, int dim) {
    // if we only have a single elements in the array
    // no need to do anything
    if (end - start < 2) return;

    int pivotIndex = randomizedPartition(originalVector, start, end, dim);

    int k = pivotIndex - start;

    if (k == i) return;
    
    if (i < k) {
        randomizedSelect(originalVector, start, pivotIndex, i, dim);
    } else {
        randomizedSelect(originalVector, pivotIndex + 1, end, i-k-1, dim);
    } 
}
#pragma endregion