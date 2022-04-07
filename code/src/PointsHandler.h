#pragma once
#include "camera.h"
#include "CameraForMapping.h"

class Dso;

class PointsHandler{
  public:

    // ********** members **********
    std::shared_ptr<Dso> dso_;

    // ********** constructor **********
    PointsHandler(Dso* dso ):
    dso_(dso)
    {};


    // ********** methods **********
    void sampleCandidates();
    void showCandidates();
    void trackCandidates();
  protected:
};
