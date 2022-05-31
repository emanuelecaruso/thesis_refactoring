#pragma once
#include "camera.h"
#include "CamCouple.h"
#include "LinSystemBA.h"

class Dso;

class MarginalizationHandler{
  public:
    // ********** members **********
    std::vector<PriorMeas*> prior_measurements_;
    Eigen::MatrixXf H_tilde_;
    Eigen::VectorXf b_tilde_;

    std::vector<CameraForMapping*> keyframes_with_priors_;
    LinSysBlocks* lin_sys_increment_;

    // ********** constructor **********
    MarginalizationHandler(Dso* dso):
    lin_sys_increment_( new LinSysBlocks(dso) ){
      H_tilde_.resize(0,0);
      b_tilde_.resize(0);
    };

    // ********** methods **********
    void removeKeyframeWithPriors(CameraForMapping* keyframe);
    void addKeyframeWithPriors(CameraForMapping* keyframe);
    void setCamIdxs();
    void loadPriorsInLinSys();
    void uploadHBTilde();
  protected:
    float loadPriorInLinSys(PriorMeas* prior_meas);
    void removeCamFromHtilde(int idx);
    void removeCamFromBtilde(int idx);
    void addCamToHtilde();
    void addCamToBtilde();


};

class BundleAdj{
  public:

    // ********** members **********
    Dso* dso_;
    int n_points_non_valid_;
    int n_points_occlusions_;
    int n_points_removed_;
    int n_points_marginalized_;
    CamCoupleContainer* cam_couple_container_;
    MarginalizationHandler* marginalization_handler_;

    // ********** constructor **********
    // BundleAdj(Dso* dso ){}
    BundleAdj(Dso* dso ):
    dso_(dso),
    n_points_non_valid_(0),
    n_points_occlusions_(0),
    n_points_removed_(0),
    n_points_marginalized_(0),
    cam_couple_container_( new CamCoupleContainer(dso_,ALL_KFS_ON_ALL_KFS ) ),
    marginalization_handler_( new MarginalizationHandler(dso_) )
    {};

    // ********** methods **********
    void optimize(bool only_pts=false);
    void marginalize();
    void setCamData();
    bool getMeasurements(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector);
    bool getMeasurementsInit(ActivePoint* active_point, int i, std::vector<MeasBA*>* measurement_vector);

    bool marginalizePoint(ActivePoint* active_point, CamCoupleContainer* cam_couple_container );
    void marginalizeKeyframe(CameraForMapping* keyframe);
    bool createPrior(ActivePoint* active_point, std::shared_ptr<CamCouple> cam_couple);

  protected:
    void marginalizePointsAndKeyframes();
    void updateState(LinSysBA& lin_sys_ba, bool only_pts=false);
    void updateBMarg(LinSysBA& lin_sys_ba);
    bool marginalizeOcclusionsInLastKeyframe();
};

class CamHkuCouple{

  // ********** members **********
  Eigen::Matrix<float,J_SZ,1> H_ku_;
  CameraForMapping* cam_;

  // ********** constructor **********
  CamHkuCouple(Eigen::Matrix<float,J_SZ,1>& H_ku, CameraForMapping* cam):
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

    ~PtDataForBA(){
      for (CamHkuCouple* a : cam_Hku_couples_)
        delete a;
    }
};

class CamDataForBA{
  public:
    // ********** members **********
    int c_idx_;
    int c_marg_idx_;
    bool has_prior_;
    // marginalization terms
    Vector6f b_k_;
    Eigen::Matrix<float,J_SZ,J_SZ> H_kk_;
    // linearization point
    Eigen::Isometry3f frame_camera_wrt_world_0_;
    Eigen::Isometry3f frame_world_wrt_camera_0_;
    float a_exposure_0_;
    float b_exposure_0_;

    // ********** constructor **********
    CamDataForBA():
    c_idx_(-1),
    c_marg_idx_(-1),
    has_prior_(false)
    {
      b_k_.setZero();
      H_kk_.setZero();
    }

};
