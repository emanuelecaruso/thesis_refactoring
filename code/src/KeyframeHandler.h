#pragma once
#include "camera.h"

class Dso;

class KeyframeHandler{
  public:
    // ********** members **********
    std::shared_ptr<Dso> dso_;

    // ********** constructor **********
    KeyframeHandler(Dso* dso):
    dso_(dso){};

    // ********** methods **********
    void addKeyframe(bool fixed);
};
