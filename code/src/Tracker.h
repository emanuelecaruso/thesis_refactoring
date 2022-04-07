#pragma once
#include "camera.h"

class Dso;

class Tracker{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;

    // ********** constructor **********
    Tracker(Dso* dso):
    dso_(dso){};

    // ********** methods **********
    void trackCam(bool groundtruth);
};
