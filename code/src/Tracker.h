#pragma once
#include "camera.h"

class Dso;

class Tracker{
  public:
    // ********** members **********
    Dso* dso_;

    // ********** constructor **********
    Tracker(Dso* dso):
    dso_(dso){};

    // ********** methods **********
    void trackCam(bool groundtruth);
};
