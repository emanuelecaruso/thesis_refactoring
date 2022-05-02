#pragma once
#include "camera.h"

class Dso;

class BundleAdj{
  public:

    // ********** members **********
    std::shared_ptr<Dso> dso_;

    // ********** constructor **********
    // BundleAdj(Dso* dso ){}
    BundleAdj(Dso* dso ):
    dso_(dso)
    {};

    // ********** methods **********

};
