#pragma once
#include "camera.h"
#include "CamCouple.h"
#include "LinSystemBA.h"

class Dso;


class BundleAdj{
  public:

    // ********** members **********
    Dso* dso_;
    CamCoupleContainer* cam_couple_container_;

    // ********** constructor **********
    // BundleAdj(Dso* dso ){}
    BundleAdj(Dso* dso ):
    dso_(dso),
    cam_couple_container_( new CamCoupleContainer(dso_,ALL_KFS_ON_ALL_KFS ) )
    {};

    // ********** methods **********
    void optimize();
    void setCamData();
    bool getMeasurements(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector);


};

class DataForBA{
  public:
    // ********** members **********
    int c_idx_;
    // std::vector<> // vector of marginalized measurements

    // ********** constructor **********
    DataForBA():
    c_idx_(-1){}

};
