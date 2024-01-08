#include <catch2/catch_test_macros.hpp>
#include "PhotonMap.hpp"

TEST_CASE("randomized partition", "[randomizedSelect]") {
    std::vector<Photon*> photonList;

    for (int i =0; i<20; i++) {
        Photon* newPhoton = new Photon();
        newPhoton->pos = Vector3f(0, 0, rand());

        photonList.push_back(newPhoton);
    }

    int j = rand() % 20;

    randomizedSelect(photonList, 0, 20, j, 2);

    Photon* pivot = photonList[j];

    for (int i=0; i<j; i++) {
        float a = photonList[i]->pos[2];
        float b = pivot->pos[2];

        REQUIRE(a<b);
    }

    for (int i=j+1; i<20; i++) {
        REQUIRE(photonList[i]->pos[2] >= pivot->pos[2]);
    }

    for (auto p: photonList) {
        if (p != nullptr)
            delete p;
    }
}

TEST_CASE("photon map", "[radianceEstimate]") {
    PhotonMap pm(30, 10);

    Vector3f pos1(1, 0, 0);

    Vector3f pos2(100, 0, 0);

    Vector3f pow(1,1,1);
    Vector3f dir(1,0,0);
    Vector3f normal(-1,0,0);

    for (int i=0; i<15; i++) {
        float randomNumber = ((float)(rand() % 1000))/1000.0f;
        Vector3f toAdd(randomNumber, randomNumber, randomNumber);
        pm.store(pos1 + toAdd, dir, pow);
        pm.store(pos2 + toAdd, dir, pow);
    }

    Vector3f view_dir(-1,0,0);
    Material m(DIFFUSE, Vector3f(0.0f));
    m.Kd = Vector3f(1,1,1);

    pm.balance();

    Vector3f result = pm.radianceEstimate(pos1, normal, view_dir, &m);
    REQUIRE(result.norm() > 0);

    Vector3f result2 = pm.radianceEstimate(pos2, normal, view_dir, &m);
    REQUIRE(result2.norm() > 0);

}