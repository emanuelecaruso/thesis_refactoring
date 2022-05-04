#pragma once
#include "camera.h"
#include "PointsContainer.h"
#include "CamCouple.h"

// class Dso;

class Tracker{
  public:
    // ********** members **********
    Dso* dso_;
    std::vector<float> chi_history_;

    // ********** constructor **********
    Tracker(Dso* dso):
    dso_(dso){};

    // ********** methods **********
    void trackCam(bool groundtruth);

  protected:
    void trackCam();
    void setInitialGuess();
    bool chiUpdateAndCheck(float chi);
    bool checkConvergence(float chi);
    void showProjectedActivePoints(int level, CamCoupleContainer& cam_couple_container);

};
