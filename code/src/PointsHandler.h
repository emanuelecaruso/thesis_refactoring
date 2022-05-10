#pragma once
#include "camera.h"
#include "epline.h"
#include "CamCouple.h"
#include "PointsContainer.h"
#include "CameraForMapping.h"

class PointsHandler{
  public:

    // ********** members **********
    Dso* dso_;
    int n_cands_to_track_;
    int n_cands_removed_;
    int n_cands_repeptitive_;
    int n_cands_no_min_;
    int n_cands_tracked_;
    int n_cands_var_too_high_;

    // ********** constructor **********
    PointsHandler(Dso* dso ):
    n_cands_to_track_(0),
    n_cands_removed_(0),
    n_cands_repeptitive_(0),
    n_cands_no_min_(0),
    n_cands_var_too_high_(0),
    n_cands_tracked_(0),
    dso_(dso)
    {};


    // ********** methods **********
    void showCandidates();
    void showProjectedCandidates();
    void showActivePoints();
    void showCoarseActivePoints(int level);
    void showProjectedActivePoints(const std::string& name);
    void showProjectedActivePoints();

    bool sampleCandidates();
    void projectCandidatesOnLastFrame();
    void projectActivePointsOnLastFrame();
    void generateCoarseActivePoints();
    void projectCandidates(CameraForMapping* cam_r, CameraForMapping* cam_m );
    void projectActivePoints(CameraForMapping* cam_r, CameraForMapping* cam_m );

    void trackCandidates(bool groundtruth);
  protected:
    void trackCandidates(CameraForMapping* keyframe, CameraForMapping* last_keyframe);
    void trackCandidatesGroundtruth(CameraForMapping* keyframe);
    bool trackCandidate(Candidate* cand, std::shared_ptr<CamCouple> cam_couple);
};

class CandTracker{
  public:
    // ********** members **********
    EpipolarLine& ep_segment_;
    Candidate* cand_;
    std::shared_ptr<CamCouple> cam_couple_;
    float phase_m;
    pixelIntensity magn_m;
    Eigen::Vector2f uv_;
    pxl pixel_;

    // ********** constructor **********
    CandTracker( EpipolarLine& ep_segment,
              Candidate* cand,
              std::shared_ptr<CamCouple> cam_couple
            ):
              ep_segment_(ep_segment),
              cand_(cand),
              cam_couple_(cam_couple)
              {};


    // ********** methods **********
    int searchMin( );
    float getCostMagn(pxl& pixel_m);
    bool getPhaseCostContribute(pxl& pixel_m, Eigen::Vector2f& uv_m, float& cost_phase);
    float getStandardDeviation( );
    bool updateCand( );


};
