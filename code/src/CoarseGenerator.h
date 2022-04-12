#pragma once
#include "camera.h"

class Dso;

class CoarseGenerator{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;

    // ********** constructor **********
    CoarseGenerator(Dso* dso):
    dso_(dso){};

    // ********** methods **********
    void generateCoarseActivePoints();
    // void addKeyframe(bool fixed);
};
