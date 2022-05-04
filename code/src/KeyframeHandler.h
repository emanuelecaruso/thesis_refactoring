#pragma once
#include "camera.h"
#include "CameraForMapping.h"

class Dso;

class KeyframeHandler{
  public:
    // ********** members **********
    Dso* dso_;

    // ********** constructor **********
    KeyframeHandler(Dso* dso):
    dso_(dso){};

    // ********** methods **********
    bool addKeyframe(bool fixed);

  protected:
    float getFlowDist();
    bool marginalizeKeyframe();
    float getPercentuageMarg(CameraForMapping* keyframe);
    void marginalize(CameraForMapping* keyframe);
    float getScore(CameraForMapping* keyframe);

};
