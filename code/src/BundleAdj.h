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
    std::vector<MarginalizedPoint*> priors_;

    // ********** constructor **********
    // BundleAdj(Dso* dso ){}
    BundleAdj(Dso* dso ):
    dso_(dso),
    cam_couple_container_( new CamCoupleContainer(dso_,ALL_KFS_ON_ALL_KFS ) )
    {};

    // ********** methods **********
    void optimize();
    void marginalizePointsAndKeyframes();
    void setCamData();
    bool getMeasurements(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector);
    void marginalizePoint(ActivePoint* active_point, CamCoupleContainer* cam_couple_container);
    void marginalizeKeyframe(CameraForMapping* keyframe);
    void createPrior(ActivePoint* active_point, CamCouple* cam_couple);


};

class CamHkuCouple{

  // ********** members **********
  Eigen::Matrix<float,6,1> H_ku_;
  CameraForMapping* cam_;

  // ********** constructor **********
  CamHkuCouple(Eigen::Matrix<float,6,1>& H_ku, CameraForMapping* cam):
  H_ku_(H_ku),
  cam_(cam)
  {}

};

class PtDataForBA{
  public:
    // ********** members **********
    int p_idx_;

    float H_uu_;
    float b_u_;
    std::vector<CamHkuCouple*> cam_Hku_couples_;

    // // ********** constructor **********
    PtDataForBA():
    p_idx_(-1),
    H_uu_(0),
    b_u_(0)
    {}

};

class CamDataForBA{
  public:
    // ********** members **********
    int c_idx_;
    int c_marg_idx_;
    // marginalization terms
    Vector6f b_k_;
    Eigen::Matrix<float,6,6> H_kk_;

    // ********** constructor **********
    CamDataForBA():
    c_idx_(-1),
    c_marg_idx_(-1)
    {
      b_k_.setZero();
      H_kk_.setZero();
    }

};
